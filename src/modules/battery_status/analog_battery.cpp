#include <lib/battery/battery.h>
#include "analog_battery.h"

// Defaults to use if the parameters are not set
#if BOARD_NUMBER_BRICKS > 0
#if defined(BOARD_BATT_V_LIST) && defined(BOARD_BATT_I_LIST)
static constexpr int   DEFAULT_V_CHANNEL[BOARD_NUMBER_BRICKS] = BOARD_BATT_V_LIST;
static constexpr int   DEFAULT_I_CHANNEL[BOARD_NUMBER_BRICKS] = BOARD_BATT_I_LIST;
#else
static constexpr int   DEFAULT_V_CHANNEL[BOARD_NUMBER_BRICKS] = {0};
static constexpr int   DEFAULT_I_CHANNEL[BOARD_NUMBER_BRICKS] = {0};
#endif
#else
static constexpr int DEFAULT_V_CHANNEL[1] = {0};
static constexpr int DEFAULT_I_CHANNEL[1] = {0};
#endif

static constexpr float DEFAULT_VOLTS_PER_COUNT = 3.3f / 4096.0f;

AnalogBattery::AnalogBattery(int index, ModuleParams *parent) :
	Battery(index, parent)
{
	char param_name[17];

	_analog_param_handles.cnt_v_volt = param_find("BAT_CNT_V_VOLT");

	_analog_param_handles.cnt_v_curr = param_find("BAT_CNT_V_CURR");

	_analog_param_handles.v_offs_cur = param_find("BAT_V_OFFS_CURR");

	snprintf(param_name, sizeof(param_name), "BAT%d_V_DIV", index);
	_analog_param_handles.v_div = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "BAT%d_A_PER_V", index);
	_analog_param_handles.a_per_v = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "BAT%d_ADC_CHANNEL", index);
	_analog_param_handles.adc_channel = param_find(param_name);

	_analog_param_handles.v_div_old = param_find("BAT_V_DIV");
	_analog_param_handles.a_per_v_old = param_find("BAT_A_PER_V");
	_analog_param_handles.adc_channel_old = param_find("BAT_ADC_CHANNEL");
}

void
AnalogBattery::updateBatteryStatusRawADC(hrt_abstime timestamp, int32_t voltage_raw, int32_t current_raw,
		bool selected_source, int priority, float throttle_normalized)
{
	float voltage_v = (voltage_raw * _analog_params.cnt_v_volt) * _analog_params.v_div;
	float current_a = ((current_raw * _analog_params.cnt_v_curr) - _analog_params.v_offs_cur) * _analog_params.a_per_v;

	bool connected = voltage_v > BOARD_ADC_OPEN_CIRCUIT_V &&
			 (BOARD_ADC_OPEN_CIRCUIT_V <= BOARD_VALID_UV || is_valid());


	Battery::updateBatteryStatus(timestamp, voltage_v, current_a, connected,
				     selected_source, priority, throttle_normalized, _params.source == 0);
}

bool AnalogBattery::is_valid()
{
#ifdef BOARD_BRICK_VALID_LIST
	bool valid[BOARD_NUMBER_BRICKS] = BOARD_BRICK_VALID_LIST;
	return valid[_index - 1];
#else
	// TODO: Maybe return false instead?
	return true;
#endif
}

int AnalogBattery::get_voltage_channel()
{
	if (_analog_params.adc_channel >= 0) {
		return _analog_params.adc_channel;

	} else {
		return DEFAULT_V_CHANNEL[_index - 1];
	}
}

int AnalogBattery::get_current_channel()
{
	// TODO: Possibly implement parameter for current sense channel
	return DEFAULT_I_CHANNEL[_index - 1];
}

void
AnalogBattery::updateParams()
{
	if (_index == 1) {
		migrateParam<float>(_analog_param_handles.v_div_old, _analog_param_handles.v_div, &_analog_params.v_div_old,
				    &_analog_params.v_div, _first_parameter_update);
		migrateParam<float>(_analog_param_handles.a_per_v_old, _analog_param_handles.a_per_v, &_analog_params.a_per_v_old,
				    &_analog_params.a_per_v, _first_parameter_update);
		migrateParam<int>(_analog_param_handles.adc_channel_old, _analog_param_handles.adc_channel,
				  &_analog_params.adc_channel_old, &_analog_params.adc_channel, _first_parameter_update);

	} else {
		param_get(_analog_param_handles.v_div, &_analog_params.v_div);
		param_get(_analog_param_handles.a_per_v, &_analog_params.a_per_v);
		param_get(_analog_param_handles.adc_channel, &_analog_params.adc_channel);
	}

	param_get(_analog_param_handles.cnt_v_volt, &_analog_params.cnt_v_volt);
	param_get(_analog_param_handles.cnt_v_curr, &_analog_params.cnt_v_curr);
	param_get(_analog_param_handles.v_offs_cur, &_analog_params.v_offs_cur);

	/* scaling of ADC ticks to battery voltage */
	if (_analog_params.cnt_v_volt < 0.0f) {
		/* apply scaling according to defaults if set to default */
		_analog_params.cnt_v_volt = (BOARD_ADC_POS_REF_V_FOR_VOLTAGE_CHAN / px4_arch_adc_dn_fullcount());
		param_set_no_notification(_analog_param_handles.cnt_v_volt, &_analog_params.cnt_v_volt);
	}

	/* scaling of ADC ticks to battery current */
	if (_analog_params.cnt_v_curr < 0.0f) {
		/* apply scaling according to defaults if set to default */
		_analog_params.cnt_v_curr = (BOARD_ADC_POS_REF_V_FOR_CURRENT_CHAN / px4_arch_adc_dn_fullcount());
		param_set_no_notification(_analog_param_handles.cnt_v_curr, &_analog_params.cnt_v_curr);
	}

	if (_analog_params.v_div <= 0.0f) {
		/* apply scaling according to defaults if set to default */
		_analog_params.v_div = BOARD_BATTERY1_V_DIV;
		param_set_no_notification(_analog_param_handles.v_div, &_analog_params.v_div);

		if (_index == 1) {
			_analog_params.v_div_old = BOARD_BATTERY1_V_DIV;
			param_set_no_notification(_analog_param_handles.v_div_old, &_analog_params.v_div_old);
		}
	}

	if (_analog_params.a_per_v <= 0.0f) {
		/* apply scaling according to defaults if set to default */

		_analog_params.a_per_v = BOARD_BATTERY1_A_PER_V;
		param_set_no_notification(_analog_param_handles.a_per_v, &_analog_params.a_per_v);

		if (_index == 1) {
			_analog_params.a_per_v_old = BOARD_BATTERY1_A_PER_V;
			param_set_no_notification(_analog_param_handles.a_per_v_old, &_analog_params.a_per_v_old);
		}
	}

	Battery::updateParams();
}
