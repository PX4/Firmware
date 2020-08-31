uint64 timestamp					# time since system start (microseconds)
uint32 esc_errorcount					# Number of reported errors by ESC - if supported
int32 esc_rpm						# Motor RPM, negative for reverse rotation [RPM] - if supported
float32 esc_voltage					# Voltage measured from current ESC [V] - if supported
float32 esc_current					# Current measured from current ESC [A] - if supported
uint8 esc_temperature					# Temperature measured from current ESC [degC] - if supported
uint8 esc_address					# Address of current ESC (in most cases 1-8 / must be set by driver)

uint8 esc_state					# State of ESC - depend on Vendor

uint8 failures					# Bitmask to indicate the internal ESC faults

uint8 FAILURE_NONE = 0
uint8 FAILURE_OVER_CURRENT_MASK = 1 		# (1 << 0)
uint8 FAILURE_OVER_VOLTAGE_MASK = 2 		# (1 << 1)
uint8 FAILURE_OVER_TEMPERATURE_MASK = 4 	# (1 << 2)
uint8 FAILURE_OVER_RPM_MASK = 8			# (1 << 3)
uint8 FAILURE_INCONSISTENT_CMD_MASK = 16 	# (1 << 4)  Set if ESC received an inconsistent command (i.e out of boundaries)
uint8 FAILURE_MOTOR_STUCK_MASK = 32		# (1 << 5)
uint8 FAILURE_GENERIC_MASK = 64			# (1 << 6)
