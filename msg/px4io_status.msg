uint64 timestamp		# time since system start (microseconds)

uint16 free_memory_bytes

float32 voltage_v		# Servo rail voltage in volts
float32 rssi_v			# RSSI pin voltage in volts

# PX4IO status flags (PX4IO_P_STATUS_FLAGS)
bool status_arm_sync
bool status_failsafe
bool status_fmu_initialized
bool status_fmu_ok
bool status_init_ok
bool status_outputs_armed
bool status_raw_pwm
bool status_rc_ok
bool status_rc_dsm
bool status_rc_ppm
bool status_rc_sbus
bool status_rc_st24
bool status_rc_sumd
bool status_safety_off

# PX4IO alarms (PX4IO_P_STATUS_ALARMS)
bool alarm_pwm_error
bool alarm_rc_lost

# PX4IO arming (PX4IO_P_SETUP_ARMING)
bool arming_failsafe_custom
bool arming_fmu_armed
bool arming_fmu_prearmed
bool arming_force_failsafe
bool arming_io_arm_ok
bool arming_lockdown
bool arming_termination_failsafe

uint16[8] pwm
uint16[8] pwm_disarmed
uint16[8] pwm_failsafe

uint16[8] pwm_rate_hz

uint16[18] raw_inputs
