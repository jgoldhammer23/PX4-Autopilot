/*
 * ActuatorEffectivenessVPP.cpp
 * Written by Aniruth Vishnupriyan for Texas A&M University Vertical Flight Design
 *
 * WHAT THIS FILE DOES
 * PX4's control allocator turns "I want this much roll / pitch / yaw / thrust" into
 * commands for individual actuators. Each vehicle type supplies a class (derived from
 * ActuatorEffectiveness) that describes its actuators. This class describes a
 * variable-pitch-propeller (VPP) quad:
 *
 *   - 4 ESCs  -> constant rotor RPM (they get NO roll/pitch/yaw)
 *   - 4 servos -> blade pitch ("collective") per rotor. These do ALL the attitude control.
 *
 * Output order is fixed: outputs 0-3 are the ESCs, outputs 4-7 are the servos.
 * Rotor order everywhere: FR, RL, FL, RR.
 *
 * WHAT IS DIFFERENT FROM ActuatorEffectivenessHelicopter.cpp
 * The helicopter class models ONE main rotor tilted by a 3-4 servo swashplate, plus a
 * tail actuator. This class models FOUR independent rotors, so:
 *   - the swashplate geometry, trim, servo linearization, tail actuator and motor-engage
 *     switch are all removed
 *   - roll/pitch/yaw are mixed by simple addition per rotor (no sin/cos)
 *   - it reuses the helicopter's parameters CA_HELI_THR_C0-4 and CA_HELI_PITCH_C0-4
 *
 * STATUS: mixing directions checked in simulation only. Not flown.
 * Thrust direction on the real linkage, and all gains, are UNVERIFIED.
 */

#include "ActuatorEffectivenessVPP.hpp"

// Lets us write Vector3f instead of matrix::Vector3f
using namespace matrix;

// An unnamed namespace makes the names inside private to this file.
// (Prevents clashes with other PX4 code, which matters under -Werror.)
namespace
{

// A struct is a bundle of named values. One RotorCfg describes one rotor.
// PX4 uses the body frame FRD: x = forward, y = right, z = down.
// yaw_sign = +1/-1 for the rotor's spin direction. VERIFY ALL SIGNS ON THE BENCH.
struct RotorCfg {
	float x;
	float y;
	float yaw_sign;
};

// constexpr = fixed when the code is compiled; cannot change while running.
// To change the layout you must edit this table and rebuild.
constexpr RotorCfg ROTORS[ActuatorEffectivenessVPP::NUM_ROTORS] = {
	{ +1.f, +1.f, +1.f},  // FR
	{ -1.f, -1.f, +1.f},  // RL
	{ +1.f, -1.f, -1.f},  // FL
	{ -1.f, +1.f, -1.f},  // RR
};

// Placeholder gains (collective units per unit torque command). Tune in tethered test.
// They set how far each servo moves per unit of roll / pitch / yaw demand.
constexpr float K_ROLL = 0.5f;
constexpr float K_PITCH = 0.5f;
constexpr float K_YAW = 0.2f;

} // namespace

// ---------------------------------------------------------------------------
// MODULE 1: constructor. Runs once, when the allocator starts with this airframe.
// The part after the colon is an "initializer list": it builds the ModuleParams
// parent part of the object first.
// ---------------------------------------------------------------------------
ActuatorEffectivenessVPP::ActuatorEffectivenessVPP(ModuleParams *parent) : ModuleParams(parent)
{
	// Reuse the helicopter curve parameters (they already exist in the param set)
	for (int i = 0; i < NUM_CURVE_POINTS; ++i) {
		char buf[17];
		// snprintf builds the parameter name as text: "CA_HELI_THR_C0", "CA_HELI_THR_C1", ...
		snprintf(buf, sizeof(buf), "CA_HELI_THR_C%d", i);
		// param_find only returns a HANDLE (a bookmark). It does not read the value.
		_ph.throttle_curve[i] = param_find(buf);
		snprintf(buf, sizeof(buf), "CA_HELI_PITCH_C%d", i);
		_ph.pitch_curve[i] = param_find(buf);
	}

	_ph.spoolup_time = param_find("COM_SPOOLUP_TIME");

	// Load the actual values now
	updateParams();
}

// ---------------------------------------------------------------------------
// MODULE 2: updateParams. Copies parameter values into plain member variables so
// the fast loop in updateSetpoint() does not have to look anything up.
// ---------------------------------------------------------------------------
void ActuatorEffectivenessVPP::updateParams()
{
	ModuleParams::updateParams();

	for (int i = 0; i < NUM_CURVE_POINTS; ++i) {
		// param_get(handle, &variable): the & passes the variable's address so the
		// function can write the current value into it.
		param_get(_ph.throttle_curve[i], &_throttle_curve[i]);
		param_get(_ph.pitch_curve[i], &_pitch_curve[i]);
	}

	param_get(_ph.spoolup_time, &_spoolup_time);
}

// ---------------------------------------------------------------------------
// MODULE 3: declare which actuators exist, and in what order.
// The order defines the index numbers used later in updateSetpoint().
// ---------------------------------------------------------------------------
bool ActuatorEffectivenessVPP::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	// Returning false means "nothing changed, do not rebuild".
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// Outputs 0-3: one ESC per rotor. Zero effectiveness, driven directly in updateSetpoint().
	// Vector3f{} is a 3-element vector of zeros (torque, then thrust). They are zero on
	// purpose: this class does not use PX4's linear matrix math.
	for (int i = 0; i < NUM_ROTORS; ++i) {
		configuration.addActuator(ActuatorType::MOTORS, Vector3f{}, Vector3f{});
	}

	// Outputs 4-7: one collective-pitch servo per rotor.
	// Remember where the servos start (this will be 4) so updateSetpoint() can find them.
	_first_servo_index = configuration.num_actuators_matrix[configuration.selected_matrix];

	for (int i = 0; i < NUM_ROTORS; ++i) {
		configuration.addActuator(ActuatorType::SERVOS, Vector3f{}, Vector3f{});
	}

	// true = "I have filled in the configuration"
	return true;
}

// ---------------------------------------------------------------------------
// MODULE 4: the core. Runs many times per second.
// Input : control_sp = the requested roll, pitch, yaw and thrust
// Output: actuator_sp = the value for every output (motors first, then servos)
// actuator_min / actuator_max are each output's limits (motors 0..1, servos -1..1).
// ---------------------------------------------------------------------------
void ActuatorEffectivenessVPP::updateSetpoint(const Vector<float, NUM_AXES> &control_sp, int matrix_index,
		ActuatorVector &actuator_sp, const ActuatorVector &actuator_min, const ActuatorVector &actuator_max)
{
	// Clear last cycle's saturation flags
	_sat = SaturationFlags{};

	// 0..1 ramp after arming (see spoolupProgress below)
	const float spool = spoolupProgress();

	// PX4's "down" is positive, so upward thrust is a NEGATIVE number.
	// Flip the sign and clamp to 0..1 to get a plain "how much lift" value.
	const float thrust = math::constrain(-control_sp(ControlAxis::THRUST_Z), 0.f, 1.f);

	// Constant-RPM setpoint from the throttle curve, ramped on arming, same on all four ESCs
	// interpolateN reads a value off the 5-point curve at position "thrust".
	const float rpm_sp = math::interpolateN(thrust, _throttle_curve) * spool;

	// The ESCs get NO roll/pitch/yaw. They only hold speed.
	for (int i = 0; i < NUM_ROTORS; ++i) {
		actuator_sp(i) = rpm_sp;
	}

	// Baseline collective from the pitch curve (servo range -1..1, so it can go through zero)
	// This is the same for all four rotors.
	const float collective = math::interpolateN(thrust, _pitch_curve);

	for (int i = 0; i < NUM_ROTORS; ++i) {
		// Servo i lives at output (4 + i)
		const int idx = _first_servo_index + i;

		// How much this rotor's servo responds to each axis, from its position and spin direction.
		// Roll : rotors on the right (y > 0) go the opposite way from rotors on the left.
		// Pitch: rotors at the front (x > 0) go the opposite way from rotors at the back.
		// Yaw  : a diagonal pair goes one way, the other pair goes the other way.
		const float roll_c = -ROTORS[i].y * K_ROLL;
		const float pitch_c = ROTORS[i].x * K_PITCH;
		const float yaw_c = ROTORS[i].yaw_sign * K_YAW;

		// THE MIXING: baseline collective plus a weighted sum of the three attitude demands.
		actuator_sp(idx) = collective
				   + roll_c * control_sp(ControlAxis::ROLL)
				   + pitch_c * control_sp(ControlAxis::PITCH)
				   + yaw_c * control_sp(ControlAxis::YAW);

		// Saturation check: did this servo hit its limit?
		if (actuator_sp(idx) < actuator_min(idx) || actuator_sp(idx) > actuator_max(idx)) {
			// If the servo is over its max, more demand in the direction of its coefficient is unallocatable
			// dir = +1 if we ran past the top limit, -1 if past the bottom
			const float dir = (actuator_sp(idx) > actuator_max(idx)) ? 1.f : -1.f;

			// Record which axis and direction caused it (reported by getUnallocatedControl)
			if (roll_c * dir > 0.f) { _sat.roll_pos = true; } else if (roll_c * dir < 0.f) { _sat.roll_neg = true; }

			if (pitch_c * dir > 0.f) { _sat.pitch_pos = true; } else if (pitch_c * dir < 0.f) { _sat.pitch_neg = true; }

			if (yaw_c * dir > 0.f) { _sat.yaw_pos = true; } else if (yaw_c * dir < 0.f) { _sat.yaw_neg = true; }
		}
	}
}

// ---------------------------------------------------------------------------
// MODULE 5: spool-up ramp. Returns 0 to 1 over COM_SPOOLUP_TIME seconds after arming,
// so the ESCs do not jump straight to full speed.
// ---------------------------------------------------------------------------
float ActuatorEffectivenessVPP::spoolupProgress()
{
	vehicle_status_s vs;

	// uORB is PX4's publish/subscribe messaging. update() fetches the newest status
	// message if a new one has arrived; otherwise we keep the last values.
	if (_vehicle_status_sub.update(&vs)) {
		_armed = vs.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
		_armed_time = vs.armed_time;
	}

	// Unlike the helicopter class, hold the ESC setpoint at 0 while disarmed
	if (!_armed) {
		return 0.f;
	}

	// hrt_absolute_time() is PX4's clock in microseconds; / 1e6f converts to seconds.
	const float time_since_arming = (hrt_absolute_time() - _armed_time) / 1e6f;

	// max(..., 0.1f) prevents dividing by zero if COM_SPOOLUP_TIME is 0.
	return math::constrain(time_since_arming / math::max(_spoolup_time, 0.1f), 0.f, 1.f);
}

// ---------------------------------------------------------------------------
// MODULE 6: report saturation back to the rate controller (anti-windup).
// If an axis could not be fully delivered, the controller stops growing its
// integrator for that axis. Only the sign matters, so we send +1, -1 or 0.
// (a ? b : c is a one-line if/else.)
// ---------------------------------------------------------------------------
void ActuatorEffectivenessVPP::getUnallocatedControl(int matrix_index, control_allocator_status_s &status)
{
	// Only the sign matters to the rate controller (anti-windup)
	status.unallocated_torque[0] = _sat.roll_pos ? 1.f : (_sat.roll_neg ? -1.f : 0.f);
	status.unallocated_torque[1] = _sat.pitch_pos ? 1.f : (_sat.pitch_neg ? -1.f : 0.f);
	status.unallocated_torque[2] = _sat.yaw_pos ? 1.f : (_sat.yaw_neg ? -1.f : 0.f);
	status.unallocated_thrust[2] = 0.f;
}
