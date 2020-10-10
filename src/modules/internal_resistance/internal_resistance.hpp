#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/topics/battery_status.h>
#include <uORB/topics/internal_resistance.h>

class InternalRes : public ModuleBase<InternalRes>, public ModuleParams, public px4::WorkItem
{
public:
	InternalRes();
	~InternalRes() {};

	static int task_spawn(int argc, char *argv[]);

	static int custom_command(int argc, char *argv[]);

	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

	void write_internal_resistance(float voltage_estimation_error, float internal_resistance);

	float extract_parameters();

	void estimate_v_dot(float voltage_estimation_error, float battery_sampling_period);

	float compute_voltage_estimation_error(float battery_sampling_period);

private:
	void Run() override;

	internal_resistance_s inter_res;
	battery_status_s battery_status;

	hrt_abstime battery_time_prev;

	hrt_abstime last_param_write_time;

	float r_s = 0.1f;
	float r_t = 0.05f;
	float c_t = 500.0f;
	float v_oc = 22.1f;

	float param_0 = r_s;
	float param_1 = (r_t + r_s)/(r_t*c_t);
	float param_2 = 1.0f/(r_t*c_t);
	float param_3 = v_oc/(r_t*c_t);

	float lambda = 0.8f;

	float adaptation_gain_0 = 0.0001f;
	float adaptation_gain_1 = 0.0001f;
	float adaptation_gain_2 = 0.0001f;
	float adaptation_gain_3 = 0.0001f;

	float voltage_estimation_prev = 22.0f;

	float signal_0 = 0;
	float signal_1 = 0;
	float signal_2 = -voltage_estimation_prev;

	float v_dot_estimate_prev = param_0*signal_0 + param_1*signal_1 + param_2*signal_2 + param_3;
	float current_filtered_a_prev;

	float best_prediction_error;

	bool best_prediction_error_reset = true;

	float best_prediction;

	float bat1_r_internal_prev = -1.0f;

	uORB::SubscriptionCallbackWorkItem _battery_sub{this, ORB_ID(battery_status)};

	uORB::Publication<internal_resistance_s> _internal_res_pub{ORB_ID(internal_resistance)};

	DEFINE_PARAMETERS(
        (ParamFloat<px4::params::BAT1_R_INTERNAL>) _bat1_r_internal,
	(ParamInt<px4::params::INTER_RES_EN>) _inter_res_en,
	(ParamFloat<px4::params::RIN_UPDATE_TIME>) _inter_res_update_period
    	)

};
