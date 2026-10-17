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
 * @file AzimuthEstimator.hpp
 *
 * Rotor azimuth and speed from slot and index edge times (rotor_encoder), hardware independent.
 */

#pragma once

#include <math.h>
#include <stdint.h>

#include <lib/mathlib/mathlib.h>

class AzimuthEstimator
{
public:
	/** edge state captured by the ISRs, times in microseconds */
	struct Edges {
		uint32_t slots_total{0};	// slot edges since start
		uint64_t t_last{0};		// last slot edge
		uint64_t t_window{0};		// slot edge `window` edges before the last
		uint32_t window{0};
		uint32_t slots_at_index{0};	// slots_total at the last index edge
		uint32_t index_total{0};	// index edges since start
		uint64_t index_time{0};
		uint64_t first_slot_time{0};	// first slot edge after the last index edge
		bool first_slot_pending{false};
	};

	static constexpr uint64_t STOP_TIMEOUT_US = 500000;	// no slot edge: rotor stopped

	explicit AzimuthEstimator(int slots = 36) { setSlots(slots); }

	void setSlots(int slots) { _slots = math::constrain(slots, 4, 128); }
	int slots() const { return _slots; }

	/** slot edges for the speed: a quarter revolution, averages the slot spacing error */
	uint32_t window() const { return static_cast<uint32_t>(math::max(_slots / 4, 1)); }

	void update(const Edges &e, uint64_t now)
	{
		const float slot_angle = M_TWOPI_F / static_cast<float>(_slots);
		const float since_last = 1e-6f * static_cast<float>(now - e.t_last);
		_speed = 0.f;

		if (e.window > 0 && now - e.t_last < STOP_TIMEOUT_US) {
			_speed = slot_angle * static_cast<float>(e.window) / (1e-6f * static_cast<float>(e.t_last - e.t_window));

			// slowing down: no edge for longer than the measured slot period bounds the speed
			if (since_last * _speed > slot_angle) {
				_speed = slot_angle / since_last;
			}
		}

		// once per revolution: index edge to first slot edge, slot count between index edges
		if (e.index_total != _index_seen && !e.first_slot_pending && _speed > 0.f) {
			if (_index_seen > 0 && e.slots_at_index - _slots_at_prev_index != static_cast<uint32_t>(_slots)) {
				++_count_errors;
			}

			_phase = math::constrain(1e-6f * static_cast<float>(e.first_slot_time - e.index_time) * _speed / slot_angle,
						 0.f, 1.f);
			_slots_at_prev_index = e.slots_at_index;
			_index_seen = e.index_total;
		}

		if (e.slots_total > 0) {
			// last slot edge: slots after the index edge (-1: before it), unreferenced before the first index
			const int32_t k = (e.index_total > 0) ? static_cast<int32_t>(e.slots_total - e.slots_at_index) - 1
					  : static_cast<int32_t>((e.slots_total - 1) % static_cast<uint32_t>(_slots));
			const int32_t k_rev = ((k % _slots) + _slots) % _slots;
			const float az = slot_angle * (static_cast<float>(k_rev) + _phase) + math::min(_speed * since_last, slot_angle);
			_azimuth = fmodf(az, M_TWOPI_F);
		}
	}

	float azimuth() const { return _azimuth; }	// [0, 2 pi), 0 at the index edge
	float speed() const { return _speed; }		// rad/s
	float phase() const { return _phase; }		// index edge to first slot edge [slots]
	bool valid() const { return _index_seen > 0 && _speed > 0.f; }	// azimuth referenced to the index, rotor turning
	uint32_t countErrors() const { return _count_errors; }	// slots between index edges != slots

private:
	int _slots{36};
	uint32_t _index_seen{0};
	uint32_t _slots_at_prev_index{0};
	float _phase{0.f};
	uint32_t _count_errors{0};
	float _speed{0.f};
	float _azimuth{0.f};
};
