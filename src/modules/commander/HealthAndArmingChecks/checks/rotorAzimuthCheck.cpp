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

	if (_rotor_azimuth.timestamp == 0 || now > _rotor_azimuth.timestamp + DATA_TIMEOUT) {
		/* EVENT
		 * @description
		 * No rotor azimuth data: coax_mixer has no cyclic control. Check the rotor encoder (rotor_encoder status).
		 */
		reporter.healthFailure(NavModes::All, health_component_t::motors_escs, events::ID("check_rotor_azimuth_missing"),
				       events::Log::Error, "Rotor azimuth missing");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "%sRotor azimuth missing\t",
					     context.isArmed() ? "" : "Preflight Fail: ");
		}

		_unreferenced_since = 0;
		return;
	}

	// armed, rotor turning, no index edge yet: the index track is not seen, cyclic stays off
	if (context.isArmed() && _rotor_azimuth.speed > 0.f && !_rotor_azimuth.valid) {
		if (_unreferenced_since == 0) {
			_unreferenced_since = now;
		}

	} else {
		_unreferenced_since = 0;
	}

	if (_unreferenced_since != 0 && now > _unreferenced_since + UNREFERENCED_TIMEOUT) {
		/* EVENT
		 * @description
		 * The rotor turns but the encoder has seen no index edge: coax_mixer has no cyclic control. Land and check
		 * the index track.
		 */
		reporter.healthFailure(NavModes::All, health_component_t::motors_escs,
				       events::ID("check_rotor_azimuth_unreferenced"),
				       events::Log::Critical, "Rotor azimuth not referenced");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Rotor azimuth not referenced, no cyclic\t");
		}
	}
}
