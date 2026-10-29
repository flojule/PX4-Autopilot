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


#include "rotorAzimuthCheck.hpp"

void RotorAzimuthChecks::checkAndReport(const Context &context, Report &reporter)
{
	if (!_rotor_azimuth_sub.advertised()) {
		return;
	}

	reporter.setIsPresent(health_component_t::motors_escs);
	_rotor_azimuth_sub.update(&_rotor_azimuth);
	const hrt_abstime now = hrt_absolute_time();
	// disarmed: blocks arming; armed: warning only (coax_mixer holds the motor at idle before takeoff, a mode
	// fallback in flight adds nothing without cyclic)
	const NavModes affected_modes = context.isArmed() ? NavModes::None : NavModes::All;

	if (_rotor_azimuth.timestamp == 0 || now > _rotor_azimuth.timestamp + DATA_TIMEOUT) {
		/* EVENT
		 * @description
		 * No rotor azimuth data: coax_mixer has no cyclic control. Check the rotor encoder (rotor_encoder status).
		 */
		reporter.healthFailure(affected_modes, health_component_t::motors_escs, events::ID("check_rotor_azimuth_missing"),
				       events::Log::Error, "Rotor azimuth missing");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "%sRotor azimuth missing\t",
					     context.isArmed() ? "" : "Preflight Fail: ");
		}

		_invalid_since = 0;
		return;
	}

	// armed (rotor at least at CM_MOT_IDLE) without a valid azimuth: no slot or no index edges, cyclic off
	if (context.isArmed() && !_rotor_azimuth.valid) {
		if (_invalid_since == 0) {
			_invalid_since = now;
		}

	} else {
		_invalid_since = 0;
	}

	if (_invalid_since != 0 && now > _invalid_since + INVALID_TIMEOUT) {
		/* EVENT
		 * @description
		 * Armed, but the encoder reports no rotor speed or has seen no index edge: coax_mixer has no cyclic control
		 * and holds the motor at CM_MOT_IDLE until takeoff. Disarm and check the encoder (rotor_encoder status).
		 */
		reporter.healthFailure(affected_modes, health_component_t::motors_escs,
				       events::ID("check_rotor_azimuth_invalid"),
				       events::Log::Critical, "Rotor azimuth invalid");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Rotor azimuth invalid, no cyclic\t");
		}
	}
}
