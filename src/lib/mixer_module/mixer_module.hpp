/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#pragma once

#include <board_config.h>
#include <drivers/drv_pwm_output.h>
#include <lib/mixer/MixerGroup.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/output_limit/output_limit.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/test_motor.h>

/**
 * @class OutputModuleInterface
 * Base class for an output module.
 */
class OutputModuleInterface : public px4::ScheduledWorkItem, public ModuleParams
{
public:
	static constexpr int MAX_ACTUATORS = PWM_OUTPUT_MAX_CHANNELS;

	OutputModuleInterface(const char *name, const px4::wq_config_t &config)
		: px4::ScheduledWorkItem(name, config), ModuleParams(nullptr) {}

	/**
	 * Callback to update the (physical) actuator outputs in the driver
	 * @param stop_motors if true, all motors must be stopped (if false, individual motors
	 *                    might still be stopped via outputs[i] == disarmed_value)
	 * @param outputs individual actuator outputs in range [min, max] or failsafe/disarmed value
	 * @param num_outputs number of outputs (<= max_num_outputs)
	 * @param num_control_groups_updated number of actuator_control groups updated
	 * @return if true, the update got handled, and actuator_outputs can be published
	 */
	virtual bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
				   unsigned num_outputs, unsigned num_control_groups_updated) = 0;

	/** called whenever the mixer gets updated/reset */
	virtual void mixerChanged() {};
};

/**
 * @class MixingOutput
 * This handles the mixing, arming/disarming and all subscriptions required for that.
 *
 * It can also drive the scheduling of the OutputModuleInterface (via uORB callbacks
 * to reduce output latency).
 */
class MixingOutput : public ModuleParams
{
public:
	static constexpr int MAX_ACTUATORS = OutputModuleInterface::MAX_ACTUATORS;

	enum class SchedulingPolicy {
		Disabled, ///< Do not drive scheduling (the module needs to call ScheduleOnInterval() for example)
		Auto ///< Drive scheduling based on subscribed actuator controls topics (via uORB callbacks)
	};

	/**
	 * Contructor
	 * @param max_num_outputs maximum number of supported outputs
	 * @param interface Parent module for scheduling, parameter updates and callbacks
	 * @param scheduling_policy
	 * @param support_esc_calibration true if the output module supports ESC calibration via max, then min setting
	 * @param ramp_up true if motor ramp up from disarmed to min upon arming is wanted
	 */
	MixingOutput(uint8_t max_num_outputs, OutputModuleInterface &interface, SchedulingPolicy scheduling_policy,
		     bool support_esc_calibration, bool ramp_up = true);

	~MixingOutput();

	void setDriverInstance(uint8_t instance) { _driver_instance = instance; }

	void printStatus() const;

	/**
	 * Call this regularly from Run(). It will call interface.updateOutputs().
	 * @return true if outputs were updated
	 */
	bool update();

	/**
	 * Check for subscription updates (e.g. after a mixer is loaded).
	 * Call this at the very end of Run() if allow_wq_switch
	 * @param allow_wq_switch if true
	 * @param limit_callbacks_to_primary set to only register callbacks for primary actuator controls (if used)
	 * @return true if subscriptions got changed
	 */
	bool updateSubscriptions(bool allow_wq_switch, bool limit_callbacks_to_primary = false);

	/**
	 * unregister uORB subscription callbacks
	 */
	void unregister();

	void setMaxTopicUpdateRate(unsigned max_topic_update_interval_us);

	/**
	 * Reset (unload) the complete mixer, called from another thread.
	 * This is thread-safe, as long as only one other thread at a time calls this.
	 */
	void resetMixerThreadSafe();

	void resetMixer();

	/**
	 * Load (append) a new mixer from a buffer, called from another thread.
	 * This is thread-safe, as long as only one other thread at a time calls this.
	 * @return 0 on success, <0 error otherwise
	 */
	int loadMixerThreadSafe(const char *buf, unsigned len);

	int loadMixer(const char *buf, unsigned len);

	const actuator_armed_s &armed() const { return _armed; }

	MixerGroup *mixers() const { return _mixers; }

	void setAllFailsafeValues(uint16_t value);
	void setAllDisarmedValues(uint16_t value);
	void setAllMinValues(uint16_t value);
	void setAllMaxValues(uint16_t value);

	uint16_t &reverseOutputMask() { return _reverse_output_mask; }
	uint16_t &failsafeValue(int index) { return _failsafe_value[index]; }
	/** Disarmed values: disarmedValue < minValue needs to hold */
	uint16_t &disarmedValue(int index) { return _disarmed_value[index]; }
	uint16_t &minValue(int index) { return _min_value[index]; }
	uint16_t &maxValue(int index) { return _max_value[index]; }

	/**
	 * Get the motor index that maps from PX4 convention to the configured one
	 * @param index motor index in [0, num_motors-1]
	 * @return reordered motor index. When out of range, the input index is returned
	 */
	int reorderedMotorIndex(int index) const;

	void setIgnoreLockdown(bool ignore_lockdown) { _ignore_lockdown = ignore_lockdown; }

	void setMaxNumOutputs(uint8_t max_num_outputs) { _max_num_outputs = max_num_outputs; }

protected:
	void updateParams() override;

private:
	void handleCommands();

	bool armNoThrottle() const
	{
		return (_armed.prearmed && !_armed.armed) || _armed.in_esc_calibration_mode;
	}

	unsigned motorTest();

	void updateOutputSlewrateMultirotorMixer();
	void updateOutputSlewrateSimplemixer();
	void setAndPublishActuatorOutputs(unsigned num_outputs, actuator_outputs_s &actuator_outputs);
	void publishMixerStatus(const actuator_outputs_s &actuator_outputs);
	void updateLatencyPerfCounter(const actuator_outputs_s &actuator_outputs);

	static int controlCallback(uintptr_t handle, uint8_t control_group, uint8_t control_index, float &input);

	enum class MotorOrdering : int32_t {
		PX4 = 0,
		Betaflight = 1
	};

	struct Command {
		enum class Type : int {
			None,
			resetMixer,
			loadMixer
		};
		px4::atomic<int> command{(int)Type::None};
		const char *mixer_buf;
		unsigned mixer_buf_length;
		int result;
	};
	Command _command; ///< incoming commands (from another thread)

	/**
	 * Reorder outputs according to _param_mot_ordering
	 * @param values values to reorder
	 */
	inline void reorderOutputs(uint16_t values[MAX_ACTUATORS]);

	void lock() { do {} while (px4_sem_wait(&_lock) != 0); }
	void unlock() { px4_sem_post(&_lock); }

	px4_sem_t _lock; /**< lock to protect access to work queue changes (includes ScheduleNow calls from another thread) */

	uint16_t _failsafe_value[MAX_ACTUATORS] {};
	uint16_t _disarmed_value[MAX_ACTUATORS] {};
	uint16_t _min_value[MAX_ACTUATORS] {};
	uint16_t _max_value[MAX_ACTUATORS] {};
	uint16_t _current_output_value[MAX_ACTUATORS] {}; ///< current output values (reordered)
	uint16_t _reverse_output_mask{0}; ///< reverses the interval [min, max] -> [max, min], NOT motor direction
	output_limit_t _output_limit;

	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};
	uORB::SubscriptionCallbackWorkItem _control_subs[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];

	uORB::PublicationMulti<actuator_outputs_s> _outputs_pub{ORB_ID(actuator_outputs)};
	uORB::PublicationMulti<multirotor_motor_limits_s> _to_mixer_status{ORB_ID(multirotor_motor_limits)}; 	///< mixer status flags

	actuator_controls_s _controls[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS] {};
	actuator_armed_s _armed{};

	hrt_abstime _time_last_dt_update_multicopter{0};
	hrt_abstime _time_last_dt_update_simple_mixer{0};
	unsigned _max_topic_update_interval_us{0}; ///< max _control_subs topic update interval (0=unlimited)

	bool _throttle_armed{false};
	bool _ignore_lockdown{false}; ///< if true, ignore the _armed.lockdown flag (for HIL outputs)

	MixerGroup *_mixers{nullptr};
	uint32_t _groups_required{0};
	uint32_t _groups_subscribed{1u << 31}; ///< initialize to a different value than _groups_required and outside of (1 << NUM_ACTUATOR_CONTROL_GROUPS)

	const SchedulingPolicy _scheduling_policy;
	const bool _support_esc_calibration;

	bool _wq_switched{false};
	uint8_t _driver_instance{0}; ///< for boards that supports multiple outputs (e.g. PX4IO + FMU)
	uint8_t _max_num_outputs;

	struct MotorTest {
		uORB::Subscription test_motor_sub{ORB_ID(test_motor)};
		bool in_test_mode{false};
		hrt_abstime timeout{0};
	};
	MotorTest _motor_test;

	OutputModuleInterface &_interface;

	perf_counter_t _control_latency_perf;

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::MC_AIRMODE>) _param_mc_airmode,   ///< multicopter air-mode
		(ParamFloat<px4::params::MOT_SLEW_MAX>) _param_mot_slew_max,
		(ParamFloat<px4::params::THR_MDL_FAC>) _param_thr_mdl_fac, ///< thrust to motor control signal modelling factor
		(ParamInt<px4::params::MOT_ORDERING>) _param_mot_ordering

	)
};
