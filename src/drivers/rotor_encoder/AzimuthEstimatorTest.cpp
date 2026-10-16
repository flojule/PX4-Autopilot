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
 * @file AzimuthEstimatorTest.cpp
 *
 * AzimuthEstimator on a simulated slot and index encoder: azimuth against the true rotor angle.
 */

#include <gtest/gtest.h>

#include <cmath>

#include "AzimuthEstimator.hpp"

namespace
{

constexpr double TWO_PI = 2.0 * M_PI;
constexpr double RPM = TWO_PI / 60.0;

/** encoder disc and the rotor_encoder ISRs: index edge at angle 0, slot j at (j + offset) slots */
class EncoderSim
{
public:
	EncoderSim(int slots, double offset) : _slots(slots), _offset(offset) {}

	/** rotor angle theta [rad, monotonic] at time t [us]: edges crossed since the last call */
	void advance(uint64_t t, double theta)
	{
		const double slot = TWO_PI / _slots;
		const auto edge_time = [&](double angle) {
			return _t + static_cast<uint64_t>(llround((angle - _theta) / (theta - _theta) * static_cast<double>(t - _t)));
		};

		if (theta > _theta) {
			// edges in angle order: index edges at n 2 pi, slot edges at (m + offset) slot
			double next_index = std::floor(_theta / TWO_PI) * TWO_PI + TWO_PI;
			double next_slot = (std::floor(_theta / slot - _offset) + 1.0 + _offset) * slot;

			while (next_index <= theta || next_slot <= theta) {
				if (next_index <= next_slot) {
					index_edge(edge_time(next_index));
					next_index += TWO_PI;

				} else {
					if (_drop_next_slot) {
						_drop_next_slot = false;

					} else {
						slot_edge(edge_time(next_slot));
					}

					next_slot += slot;
				}
			}
		}

		_t = t;
		_theta = theta;
	}

	/** lose the next slot edge (dirt, missed interrupt) */
	void dropNextSlot() { _drop_next_slot = true; }

	AzimuthEstimator::Edges edges(uint32_t window_max) const
	{
		AzimuthEstimator::Edges e;
		e.slots_total = _slots_total;
		e.window = _slots_total > 0 ? std::min(_slots_total - 1, window_max) : 0;
		e.t_last = _slot_time[(_slots_total - 1) % RING];
		e.t_window = _slot_time[(_slots_total - 1 - e.window) % RING];
		e.slots_at_index = _slots_at_index;
		e.index_total = _index_total;
		e.index_time = _index_time;
		e.first_slot_time = _first_slot_time;
		e.first_slot_pending = _first_slot_pending;
		return e;
	}

private:
	static constexpr unsigned RING = 64;

	void slot_edge(uint64_t t)
	{
		_slot_time[_slots_total % RING] = t;
		++_slots_total;

		if (_first_slot_pending) {
			_first_slot_time = t;
			_first_slot_pending = false;
		}
	}

	void index_edge(uint64_t t)
	{
		_index_time = t;
		_slots_at_index = _slots_total;
		++_index_total;
		_first_slot_pending = true;
	}

	int _slots;
	double _offset;
	uint64_t _t{0};
	double _theta{0.0};
	bool _drop_next_slot{false};

	uint64_t _slot_time[RING] {};
	uint32_t _slots_total{0};
	uint32_t _slots_at_index{0};
	uint32_t _index_total{0};
	uint64_t _index_time{0};
	uint64_t _first_slot_time{0};
	bool _first_slot_pending{false};
};

/** wrapped azimuth error [deg] */
double error_deg(float azimuth, double theta)
{
	return std::remainder(static_cast<double>(azimuth) - theta, TWO_PI) * 180.0 / M_PI;
}

struct Result {
	double max_error_deg{0.0};
	double max_speed_error{0.0};	// relative
};

/**
 * Rotor angle from speed(t) [rad/s], sampled every 10 us; estimator at 400 Hz from t_check [s] to
 * t_end [s] (errors only while the true speed is above 100 rpm)
 */
template<typename Speed>
Result simulate(AzimuthEstimator &est, EncoderSim &enc, Speed speed, double t_check, double t_end,
	     double t_drop_slot = -1.0)
{
	Result run;
	double theta = 0.0;
	int step = 0;

	for (uint64_t t = 10; t <= static_cast<uint64_t>(t_end * 1e6); t += 10, ++step) {
		const double om = speed(1e-6 * static_cast<double>(t));
		theta += om * 1e-5;
		if (step == static_cast<int>(t_drop_slot * 1e5)) {
			enc.dropNextSlot();
		}

		enc.advance(t, theta);

		if (t % 2500 == 0) {
			est.update(enc.edges(est.window()), t);

			if (1e-6 * static_cast<double>(t) >= t_check && om > 100.0 * RPM) {
				run.max_error_deg = std::max(run.max_error_deg, std::fabs(error_deg(est.azimuth(), theta)));
				run.max_speed_error = std::max(run.max_speed_error, std::fabs(static_cast<double>(est.speed()) / om - 1.0));
			}
		}
	}

	return run;
}

} // namespace

TEST(AzimuthEstimatorTest, ConstantSpeed)
{
	for (double rpm : {131.0, 313.0, 450.0}) {
		AzimuthEstimator est(36);
		EncoderSim enc(36, 0.37);
		const Result run = simulate(est, enc, [&](double) { return rpm * RPM; }, 1.0, 3.0);
		EXPECT_LT(run.max_error_deg, 0.05) << rpm << " rpm";
		EXPECT_LT(run.max_speed_error, 1e-3) << rpm << " rpm";
		EXPECT_NEAR(est.phase(), 0.37f, 1e-3f);
		EXPECT_EQ(est.countErrors(), 0u);
	}
}

TEST(AzimuthEstimatorTest, SpinUp)
{
	// 0 to 450 rpm at 5 rad/s^2 (9.4 s): window-average speed lags the acceleration
	AzimuthEstimator est(36);
	EncoderSim enc(36, 0.6);
	const Result run = simulate(est, enc, [](double t) { return std::min(5.0 * t, 450.0 * RPM); }, 0.0, 12.0);
	EXPECT_LT(run.max_error_deg, 1.0);
	EXPECT_LT(run.max_speed_error, 0.05);
	EXPECT_EQ(est.countErrors(), 0u);
}

TEST(AzimuthEstimatorTest, IndexOnSlotEdge)
{
	// index and slot edges coincide: either order per revolution, same azimuth
	for (double offset : {1e-4, 0.9999}) {
		AzimuthEstimator est(36);
		EncoderSim enc(36, offset);
		const Result run = simulate(est, enc, [](double) { return 313.0 * RPM; }, 1.0, 3.0);
		EXPECT_LT(run.max_error_deg, 0.05) << offset;
		EXPECT_EQ(est.countErrors(), 0u);
	}
}

TEST(AzimuthEstimatorTest, MissedSlotCountedAndReferenced)
{
	// one slot edge lost at 1 s: one count error, azimuth re-referenced at the next index edge
	AzimuthEstimator est(36);
	EncoderSim enc(36, 0.37);
	const Result run = simulate(est, enc, [](double) { return 313.0 * RPM; }, 1.5, 3.0, 1.0);
	EXPECT_EQ(est.countErrors(), 1u);
	EXPECT_LT(run.max_error_deg, 0.05);
}

TEST(AzimuthEstimatorTest, Stop)
{
	// 313 rpm, then stopped at 2 s: speed bound by the time since the last edge, zero after the timeout
	AzimuthEstimator est(36);
	EncoderSim enc(36, 0.37);
	simulate(est, enc, [](double t) { return t < 2.0 ? 313.0 * RPM : 0.0; }, 0.0, 2.1);
	EXPECT_LT(est.speed(), 313.f * static_cast<float>(RPM) * 0.2f);

	AzimuthEstimator est_late(36);
	EncoderSim enc_late(36, 0.37);
	simulate(est_late, enc_late, [](double t) { return t < 2.0 ? 313.0 * RPM : 0.0; }, 0.0, 2.6);
	EXPECT_FLOAT_EQ(est_late.speed(), 0.f);
	EXPECT_LT(est_late.azimuth(), static_cast<float>(TWO_PI));
}
