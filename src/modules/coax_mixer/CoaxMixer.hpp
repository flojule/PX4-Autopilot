/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * Coaxial per-blade mixer (aperocopter): replaces control_allocator.
 * Runs on every rotor_azimuth update; outputs motor 1 (governed rpm) and servos 1-4 (blade pitch).
 */

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/perf/perf_counter.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rotor_azimuth.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

using namespace time_literals;

class CoaxMixer : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	CoaxMixer();
	~CoaxMixer() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	void Run() override;

	uORB::Publication<actuator_motors_s> _motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<actuator_servos_s> _servos_pub{ORB_ID(actuator_servos)};

	uORB::SubscriptionCallbackWorkItem _rotor_azimuth_sub{this, ORB_ID(rotor_azimuth)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _thrust_sp_sub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Subscription _torque_sp_sub{ORB_ID(vehicle_torque_setpoint)};

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	bool _armed{false};
	float _thrust{0.f};           // [0, 1] upward
	float _torque[3] {};          // normalized roll, pitch, yaw
	float _pitch_cmd[4] {};       // last blade pitch commands [rad]

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::CM_COLL_MIN>) _param_coll_min,
		(ParamFloat<px4::params::CM_COLL_MAX>) _param_coll_max,
		(ParamFloat<px4::params::CM_CYC_MAX>) _param_cyc_max,
		(ParamFloat<px4::params::CM_DIFF_MAX>) _param_diff_max,
		(ParamFloat<px4::params::CM_PITCH_MIN>) _param_pitch_min,
		(ParamFloat<px4::params::CM_PITCH_MAX>) _param_pitch_max,
		(ParamFloat<px4::params::CM_RPM>) _param_rpm,
		(ParamFloat<px4::params::CM_RPM_MAX>) _param_rpm_max,
		(ParamFloat<px4::params::CM_SRV_TAU>) _param_srv_tau,
		(ParamFloat<px4::params::CM_SRV_RATE>) _param_srv_rate,
		(ParamBool<px4::params::CM_LAG_COMP>) _param_lag_comp,
		(ParamFloat<px4::params::CM_AZ_OFF>) _param_az_off
	)
};
