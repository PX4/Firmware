uint64 timestamp		# time since system start (microseconds)

uint16 free_memory_bytes

float32 voltage_v		# Servo rail voltage in volts
float32 rssi_v			# RSSI pin voltage in volts

# PX4IO status flags (PX4IO_P_STATUS_FLAGS)
bool status_outputs_armed
bool status_override
bool status_rc_ok
bool status_rc_ppm
bool status_rc_dsm
bool status_rc_sbus
bool status_fmu_ok
bool status_raw_pwm
bool status_mixer_ok
bool status_arm_sync
bool status_init_ok
bool status_failsafe
bool status_safety_off
bool status_fmu_initialized
bool status_rc_st24
bool status_rc_sumd

# PX4IO alarms (PX4IO_P_STATUS_ALARMS)
bool alarm_vbatt_low
bool alarm_temperature
bool alarm_servo_current
bool alarm_acc_current
bool alarm_fmu_lost
bool alarm_rc_lost
bool alarm_pwm_error
bool alarm_vservo_fault

# PX4IO arming (PX4IO_P_SETUP_ARMING)
bool arming_io_arm_ok
bool arming_fmu_armed
bool arming_fmu_prearmed
bool arming_manual_override_ok
bool arming_failsafe_custom
bool arming_inair_restart_ok
bool arming_always_pwm_enable
bool arming_rc_handling_disabled
bool arming_lockdown
bool arming_force_failsafe
bool arming_termination_failsafe
bool arming_override_immediate


int16[8] actuators
uint16[8] servos

uint16[18] raw_inputs
