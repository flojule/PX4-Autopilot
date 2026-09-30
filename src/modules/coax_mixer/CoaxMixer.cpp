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

#include "CoaxMixer.hpp"

#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <plant/mixer.hpp>

ModuleBase::Descriptor CoaxMixer::desc{task_spawn, custom_command, print_usage};

CoaxMixer::CoaxMixer() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
}

CoaxMixer::~CoaxMixer()
{
	perf_free(_loop_perf);
}

bool CoaxMixer::init()
{
	if (!_rotor_azimuth_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void CoaxMixer::Run()
{
	if (should_exit()) {
		_rotor_azimuth_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_loop_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);
		updateParams();
	}

	actuator_armed_s armed;

	if (_armed_sub.update(&armed)) {
		_armed = armed.armed;
	}

	vehicle_thrust_setpoint_s thrust_sp;

	if (_thrust_sp_sub.update(&thrust_sp)) {
		_thrust = math::constrain(-thrust_sp.xyz[2], 0.f, 1.f);
	}

	vehicle_torque_setpoint_s torque_sp;

	if (_torque_sp_sub.update(&torque_sp)) {
		for (int i = 0; i < 3; i++) {
			_torque[i] = math::constrain(torque_sp.xyz[i], -1.f, 1.f);
		}
	}

	rotor_azimuth_s az;

	if (_rotor_azimuth_sub.update(&az)) {
		// hub moment: M_x = -K (theta1s_u + theta1s_l), M_y = K (theta1c_u + theta1c_l); yaw = Q_l - Q_u
		const float coll = math::radians(_param_coll_min.get()
						  + _thrust * (_param_coll_max.get() - _param_coll_min.get()));
		const float diff = math::radians(_param_diff_max.get()) * _torque[2];
		const float cyc = math::radians(_param_cyc_max.get());
		const float c = cyc * _torque[1];
		const float s = -cyc * _torque[0];
		const std::array<double, 6> u{coll - diff, coll + diff, c, c, s, s};
		const auto cmd = plant::mix(u, az.azimuth + math::radians(_param_az_off.get()), az.speed,
					    _param_srv_tau.get(), math::radians(_param_srv_rate.get()), _param_lag_comp.get());

		const float pmin = math::radians(_param_pitch_min.get());
		const float pmax = math::radians(_param_pitch_max.get());
		actuator_servos_s servos{};

		for (int k = 0; k < 4; k++) {
			_pitch_cmd[k] = static_cast<float>(cmd[k]);
			servos.control[k] = math::constrain(2.f * (_pitch_cmd[k] - pmin) / (pmax - pmin) - 1.f, -1.f, 1.f);
		}

		for (int k = 4; k < actuator_servos_s::NUM_CONTROLS; k++) {
			servos.control[k] = NAN;
		}

		actuator_motors_s motors{};

		for (int k = 0; k < actuator_motors_s::NUM_CONTROLS; k++) {
			motors.control[k] = NAN;
		}

		if (_armed) {
			motors.control[0] = math::constrain(_param_rpm.get() / _param_rpm_max.get(), 0.f, 1.f);
		}

		const hrt_abstime now = hrt_absolute_time();
		servos.timestamp_sample = motors.timestamp_sample = az.timestamp;
		servos.timestamp = motors.timestamp = now;
		_servos_pub.publish(servos);
		_motors_pub.publish(motors);
	}

	perf_end(_loop_perf);
}

int CoaxMixer::task_spawn(int argc, char *argv[])
{
	CoaxMixer *instance = new CoaxMixer();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;
	return PX4_ERROR;
}

int CoaxMixer::print_status()
{
	PX4_INFO("armed %d, thrust %.3f, torque %.3f %.3f %.3f", _armed, (double)_thrust, (double)_torque[0],
		 (double)_torque[1], (double)_torque[2]);
	PX4_INFO("blade pitch cmd [deg] %.2f %.2f %.2f %.2f", (double)math::degrees(_pitch_cmd[0]),
		 (double)math::degrees(_pitch_cmd[1]), (double)math::degrees(_pitch_cmd[2]), (double)math::degrees(_pitch_cmd[3]));
	perf_print_counter(_loop_perf);
	return 0;
}

int CoaxMixer::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int CoaxMixer::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Coaxial per-blade pitch mixer (aperocopter). Replaces control_allocator: maps thrust and torque
setpoints to collective, differential collective and cyclic, then to 4 blade pitch commands phased by
the rotor azimuth (rotor_azimuth), with servo lag compensation. Motor 1 holds a governed rpm when armed.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("coax_mixer", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int coax_mixer_main(int argc, char *argv[])
{
	return ModuleBase::main(CoaxMixer::desc, argc, argv);
}
