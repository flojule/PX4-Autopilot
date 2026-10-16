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
 * @file RotorEncoder.hpp
 *
 * Optical rotor azimuth encoder (aperocopter upper mast): a slot track (RENC_SLOTS slots) and an
 * index track (1 slot) on two FMU capture pins -> rotor_azimuth at 400 Hz for coax_mixer.
 */

#pragma once

#include "AzimuthEstimator.hpp"

#include <drivers/drv_hrt.h>
#include <px4_arch/micro_hal.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/rotor_azimuth.h>

using namespace time_literals;

class RotorEncoder : public ModuleBase, public px4::ScheduledWorkItem, public ModuleParams
{
public:
	static Descriptor desc;

	RotorEncoder();
	~RotorEncoder() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	bool init();

	/** interrupt-driven: stopped explicitly */
	static void stop();

private:
	static constexpr hrt_abstime PUBLISH_INTERVAL = 2500_us;	// 400 Hz, the coax_mixer rate
	static constexpr hrt_abstime SLOT_DEBOUNCE = 50_us;		// 128 slots at 450 rpm: 1 ms
	static constexpr hrt_abstime INDEX_DEBOUNCE = 10_ms;		// 450 rpm: 133 ms
	static constexpr unsigned RING = 64;				// slot edge times, > speed window + 1

	static int slot_isr(int irq, void *context, void *arg);
	static int index_isr(int irq, void *context, void *arg);

	void Run() override;

	bool allocate(int32_t function, int &channel, uint32_t &gpio, xcpt_t isr);
	void release(int channel, uint32_t gpio);

	int _slot_channel{-1};
	int _index_channel{-1};
	uint32_t _slot_gpio{0};
	uint32_t _index_gpio{0};

	// written in the ISRs, copied in Run() inside a critical section
	hrt_abstime _slot_time[RING] {};
	uint32_t _slots_total{0};		// slot edges since start
	uint32_t _slots_at_index{0};		// _slots_total at the last index edge
	uint32_t _index_total{0};		// index edges since start
	hrt_abstime _index_time{0};
	hrt_abstime _first_slot_time{0};	// first slot edge after the last index edge
	bool _first_slot_pending{false};
	uint32_t _glitches{0};			// edges rejected by the debounce

	AzimuthEstimator _estimator;

	uORB::Publication<rotor_azimuth_s> _rotor_azimuth_pub{ORB_ID(rotor_azimuth)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::RENC_SLOTS>) _param_slots
	)
};
