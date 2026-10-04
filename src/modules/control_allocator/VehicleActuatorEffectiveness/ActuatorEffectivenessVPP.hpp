/*
 * Written by Aniruth Vishnupriyan for Texas A&M University Vertical Flight Design
 */

#pragma once

#include "ActuatorEffectiveness.hpp"

#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_status.h>

/**
 * Variable-pitch quad: 4 ESCs (constant RPM) + 4 collective-pitch servos.
 *
 * Output order: motors 0-3, servos 4-7, both in rotor order FR, RL, FL, RR.
 * Allocation is non-linear, so it is done in updateSetpoint() (like the
 * helicopter class) and the effectiveness matrix only reserves the actuators.
 */
class ActuatorEffectivenessVPP : public ModuleParams, public ActuatorEffectiveness
{
public:
	static constexpr int NUM_ROTORS = 4;
	static constexpr int NUM_CURVE_POINTS = 5;

	explicit ActuatorEffectivenessVPP(ModuleParams *parent);
	virtual ~ActuatorEffectivenessVPP() = default;

	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
			    ActuatorVector &actuator_sp, const ActuatorVector &actuator_min,
			    const ActuatorVector &actuator_max) override;

	void getUnallocatedControl(int matrix_index, control_allocator_status_s &status) override;

	const char *name() const override { return "VPP"; }

protected:
	void updateParams() override;

private:
	float spoolupProgress();

	struct ParamHandles {
		param_t throttle_curve[NUM_CURVE_POINTS];
		param_t pitch_curve[NUM_CURVE_POINTS];
		param_t spoolup_time;
	};

	struct SaturationFlags {
		bool roll_pos;
		bool roll_neg;
		bool pitch_pos;
		bool pitch_neg;
		bool yaw_pos;
		bool yaw_neg;
	};

	ParamHandles _ph{};
	float _throttle_curve[NUM_CURVE_POINTS] {};
	float _pitch_curve[NUM_CURVE_POINTS] {};
	float _spoolup_time{1.f};

	int _first_servo_index{0};

	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	bool _armed{false};
	hrt_abstime _armed_time{0};

	SaturationFlags _sat{};
};
