uint64 timestamp	# time since system start (microseconds)

bool armed		# Set to true if system is armed
bool prearmed		# Set to true if the actuator safety is disabled but motors are not armed
bool ready_to_arm	# Set to true if system is ready to be armed
bool lockdown		# Set to true if actuators are forced to being disabled (due to emergency or HIL)
bool manual_lockdown    # Set to true if manual throttle kill switch is engaged
bool force_failsafe	# Set to true if the actuators are forced to the failsafe position
bool in_esc_calibration_mode # IO/FMU should ignore messages from the actuator controls topics
bool soft_stop # Set to true if we need to ESCs to remove the idle constraint
