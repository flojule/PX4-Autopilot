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


#include "RotorEncoder.hpp"

#include <board_config.h>
#include <drivers/drv_pwm_output.h>
#include <lib/mathlib/mathlib.h>
#include <parameters/param.h>
#include <px4_arch/io_timer.h>

ModuleBase::Descriptor RotorEncoder::desc{task_spawn, custom_command, print_usage};

static constexpr int32_t FUNCTION_SLOT = 2071;	// Rotor_Enc_Slot, mixer_module/output_functions.yaml
static constexpr int32_t FUNCTION_INDEX = 2072;	// Rotor_Enc_Index

RotorEncoder::RotorEncoder() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	ModuleParams(nullptr)
{
}

RotorEncoder::~RotorEncoder()
{
	ScheduleClear();
	release(_slot_channel, _slot_gpio);
	release(_index_channel, _index_gpio);
}

bool RotorEncoder::allocate(int32_t function, int &channel, uint32_t &gpio, xcpt_t isr)
{
	for (unsigned i = 0; i < PWM_OUTPUT_MAX_CHANNELS; ++i) {
		char param_name[17];
		snprintf(param_name, sizeof(param_name), "%s_%s%d", PARAM_PREFIX, "FUNC", i + 1);
		const param_t handle = param_find(param_name);
		int32_t value;

		if (handle != PARAM_INVALID && param_get(handle, &value) == 0 && value == function) {
			channel = i;
		}
	}

	if (channel < 0) {
		PX4_ERR("no %s channel (%s_FUNCn %" PRId32 ")", function == FUNCTION_SLOT ? "slot" : "index", PARAM_PREFIX,
			function);
		return false;
	}

	const int ret = io_timer_allocate_channel(channel, IOTimerChanMode_RPM);

	if (ret != PX4_OK) {
		PX4_ERR("channel %d alloc failed (%d)", channel + 1, ret);
		channel = -1;
		return false;
	}

	gpio = PX4_MAKE_GPIO_EXTI(io_timer_channel_get_as_pwm_input(channel));
	return px4_arch_gpiosetevent(gpio, true, false, true, isr, this) == PX4_OK;
}

void RotorEncoder::release(int channel, uint32_t gpio)
{
	if (channel >= 0) {
		px4_arch_gpiosetevent(gpio, false, false, false, nullptr, nullptr);
		io_timer_unallocate_channel(channel);
	}
}

bool RotorEncoder::init()
{
	_estimator.setSlots(_param_slots.get());

	if (!allocate(FUNCTION_SLOT, _slot_channel, _slot_gpio, &RotorEncoder::slot_isr)
	    || !allocate(FUNCTION_INDEX, _index_channel, _index_gpio, &RotorEncoder::index_isr)) {
		return false;
	}

	ScheduleOnInterval(PUBLISH_INTERVAL);
	return true;
}

int RotorEncoder::slot_isr(int irq, void *context, void *arg)
{
	RotorEncoder *e = static_cast<RotorEncoder *>(arg);
	const hrt_abstime t = hrt_absolute_time();

	if (e->_slots_total > 0 && t - e->_slot_time[(e->_slots_total - 1) % RING] < SLOT_DEBOUNCE) {
		++e->_glitches;
		return PX4_OK;
	}

	e->_slot_time[e->_slots_total % RING] = t;
	++e->_slots_total;

	if (e->_first_slot_pending) {
		e->_first_slot_time = t;
		e->_first_slot_pending = false;
	}

	return PX4_OK;
}

int RotorEncoder::index_isr(int irq, void *context, void *arg)
{
	RotorEncoder *e = static_cast<RotorEncoder *>(arg);
	const hrt_abstime t = hrt_absolute_time();

	if (e->_index_total > 0 && t - e->_index_time < INDEX_DEBOUNCE) {
		++e->_glitches;
		return PX4_OK;
	}

	e->_index_time = t;
	e->_slots_at_index = e->_slots_total;
	++e->_index_total;
	e->_first_slot_pending = true;
	return PX4_OK;
}

void RotorEncoder::Run()
{
	if (should_exit()) {
		exit_and_cleanup(desc);
		return;
	}

	const hrt_abstime now = hrt_absolute_time();
	AzimuthEstimator::Edges e;
	const irqstate_t flags = px4_enter_critical_section();
	e.slots_total = _slots_total;
	e.window = e.slots_total > 0 ? math::min(e.slots_total - 1, _estimator.window()) : 0;
	e.t_last = _slot_time[(e.slots_total - 1) % RING];
	e.t_window = _slot_time[(e.slots_total - 1 - e.window) % RING];
	e.slots_at_index = _slots_at_index;
	e.index_total = _index_total;
	e.index_time = _index_time;
	e.first_slot_time = _first_slot_time;
	e.first_slot_pending = _first_slot_pending;
	px4_leave_critical_section(flags);

	_estimator.update(e, now);

	rotor_azimuth_s msg{};
	msg.timestamp = now;
	msg.azimuth = _estimator.azimuth();
	msg.speed = _estimator.speed();
	_rotor_azimuth_pub.publish(msg);
}

int RotorEncoder::task_spawn(int argc, char *argv[])
{
	RotorEncoder *instance = new RotorEncoder();

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

int RotorEncoder::print_status()
{
	PX4_INFO("slots %d on channel %d, index on channel %d", _estimator.slots(), _slot_channel + 1,
		 _index_channel + 1);
	PX4_INFO("speed %.1f rpm, azimuth %.1f deg, index phase %.2f slot",
		 (double)(_estimator.speed() * 60.f / M_TWOPI_F), (double)math::degrees(_estimator.azimuth()),
		 (double)_estimator.phase());
	PX4_INFO("slot edges %" PRIu32 ", index edges %" PRIu32 ", count errors %" PRIu32 ", glitches %" PRIu32,
		 _slots_total, _index_total, _estimator.countErrors(), _glitches);
	return 0;
}

int RotorEncoder::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int RotorEncoder::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Optical rotor azimuth encoder on the upper rotor mast (aperocopter): a slot disc with RENC_SLOTS
evenly spaced slots and a second track with one index slot, read by two optical forks on FMU
capture pins (output functions "Rotor Encoder Slot" and "Rotor Encoder Index"; their timers carry
no PWM outputs). Publishes rotor_azimuth at 400 Hz for coax_mixer: speed over a quarter
revolution of slot edges, azimuth from the slot count since the index edge plus the index-to-slot
phase measured each revolution, extrapolated to the publication time. Azimuth 0 is the index edge;
coax_mixer's CM_AZ_OFF sets blade 0. Until the first index edge the azimuth is unreferenced.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rotor_encoder", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

void RotorEncoder::stop()
{
	exit_and_cleanup(desc);
}

extern "C" __EXPORT int rotor_encoder_main(int argc, char *argv[])
{
	if (argc >= 2 && !strcmp(argv[1], "stop") && RotorEncoder::is_running(RotorEncoder::desc)) {
		RotorEncoder::stop();
	}

	return ModuleBase::main(RotorEncoder::desc, argc, argv);
}
