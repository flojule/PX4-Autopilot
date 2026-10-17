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

	ScheduleDelayed(AZIMUTH_TIMEOUT);
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

	const hrt_abstime now = hrt_absolute_time();
	rotor_azimuth_s az;

	if (_rotor_azimuth_sub.update(&az)) {
		_azimuth_last = az.timestamp;
		mix(az.azimuth, az.speed, az.valid, az.timestamp, now);

	} else if (now >= _azimuth_last + AZIMUTH_TIMEOUT) {
		// no rotor azimuth (encoder lost): outputs at the watchdog rate, no cyclic, flap gains at CM_RPM_REF
		mix(0.f, 0.f, false, now, now);
	}

	// watchdog: runs again if rotor_azimuth stops
	ScheduleDelayed(AZIMUTH_TIMEOUT);

	perf_end(_loop_perf);
}

void CoaxMixer::mix(float azimuth, float speed, bool azimuth_valid, hrt_abstime timestamp_sample, hrt_abstime now)
{
	// hub moment: M_x = -K (d1s_u + d1s_l), M_y = K (d1c_u + d1c_l), yaw = Q_l - Q_u; flap moment and
	// torque per flap scale with Omega^2: normalize to CM_RPM_REF with the measured rotor speed
	const float om_ref = _param_rpm_ref.get() * M_TWOPI_F / 60.f;
	const float om = (speed > 0.f) ? math::max(speed, 0.25f * om_ref) : om_ref;
	const float scale = om_ref * om_ref / (om * om);
	const float diff = math::radians(_param_diff_max.get()) * _torque[2] * scale;
	// cyclic needs the blade azimuth: zero until the encoder is referenced and while it is lost
	_cyclic_enabled = azimuth_valid;
	const float cyc = azimuth_valid ? math::radians(_param_cyc_max.get()) * scale : 0.f;
	const float c = cyc * _torque[1];
	const float s = -cyc * _torque[0];
	const double u[6] {-diff, diff, c, c, s, s};
	const auto cmd = plant::mix(u, azimuth + math::radians(_param_az_off.get()), om,
				    _param_srv_tau.get(), math::radians(_param_srv_rate.get()));

	const float fmin = math::radians(_param_flap_min.get());
	const float fmax = math::radians(_param_flap_max.get());
	actuator_servos_s servos{};
	bool clipped = false;

	for (int k = 0; k < 4; k++) {
		_flap_cmd[k] = static_cast<float>(cmd[k]);
		clipped |= (_flap_cmd[k] < fmin) || (_flap_cmd[k] > fmax);
		servos.control[k] = math::constrain(2.f * (_flap_cmd[k] - fmin) / (fmax - fmin) - 1.f, -1.f, 1.f);
	}

	for (int k = 4; k < actuator_servos_s::NUM_CONTROLS; k++) {
		servos.control[k] = NAN;
	}

	// rate controller anti-windup (replaces control_allocator's): flap range reached, or no cyclic -> no
	// further integration along the torque setpoint
	control_allocator_status_s status{};
	status.torque_setpoint_achieved = !clipped && azimuth_valid;
	status.thrust_setpoint_achieved = true;

	for (int i = 0; i < 3; i++) {
		status.unallocated_torque[i] = clipped ? _torque[i] : 0.f;
	}

	if (!azimuth_valid) {
		status.unallocated_torque[0] = _torque[0];
		status.unallocated_torque[1] = _torque[1];
	}

	actuator_motors_s motors{};

	for (int k = 0; k < actuator_motors_s::NUM_CONTROLS; k++) {
		motors.control[k] = NAN;
	}

	if (_armed) {
		motors.control[0] = sqrtf(_thrust); // fixed blade pitch: thrust ~ Omega^2, output 1 = CM_RPM_MAX
	}

	servos.timestamp_sample = motors.timestamp_sample = timestamp_sample;
	servos.timestamp = motors.timestamp = now;
	_servos_pub.publish(servos);
	_motors_pub.publish(motors);
	status.timestamp = now;
	_allocator_status_pub.publish(status);

	if (_param_notch_esc.get() && now >= _esc_status_last + 20_ms) {
		// blade passage (2/rev) as ESC 0 rpm: gyro dynamic notch (IMU_GYRO_DNF_EN ESC RPM) on its harmonics;
		// below CM_NOTCH_MIN reported offline (notch times out): there the notch sits inside the rate loop
		const float blade_hz = speed / static_cast<float>(M_PI);
		const bool on = blade_hz >= _param_notch_min.get();
		esc_status_s esc{};
		esc.timestamp = esc.esc[0].timestamp = now;
		esc.esc_count = 1;
		esc.esc_online_flags = on ? 1 : 0;
		esc.esc[0].esc_rpm = on ? static_cast<int32_t>(blade_hz * 60.f) : 0;
		_esc_status_pub.publish(esc);
		_esc_status_last = now;
	}
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
	PX4_INFO("armed %d, thrust %.3f, torque %.3f %.3f %.3f, cyclic %s", _armed, (double)_thrust,
		 (double)_torque[0], (double)_torque[1], (double)_torque[2],
		 _cyclic_enabled ? "on" : "off (rotor azimuth invalid or lost)");
	PX4_INFO("flap cmd [deg] %.2f %.2f %.2f %.2f", (double)math::degrees(_flap_cmd[0]),
		 (double)math::degrees(_flap_cmd[1]), (double)math::degrees(_flap_cmd[2]), (double)math::degrees(_flap_cmd[3]));
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
Coaxial per-blade tip-flap mixer (aperocopter; fixed blade pitch). Replaces control_allocator: thrust
setpoint to rotor speed (motor 1, thrust ~ Omega^2), yaw torque to differential flap, roll/pitch torque
to cyclic flap, then 4 flap commands phased by the rotor azimuth (rotor_azimuth) with servo lag
compensation. Flap terms are scaled by (CM_RPM_REF / Omega)^2 for a speed-independent moment.
Cyclic is zero while rotor_azimuth is invalid (encoder not yet referenced to its index, rotor stopped)
or lost (no update for 20 ms; outputs then continue at 50 Hz).
Publishes control_allocator_status (flap range reached: rate loop anti-windup) and, with CM_NOTCH_ESC,
the blade passage rate as esc_status rpm for the gyro dynamic notch at 2/rev and 4/rev.
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
