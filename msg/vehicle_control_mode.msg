uint64 timestamp		# time since system start (microseconds)
bool flag_armed			# synonym for actuator_armed.armed

bool flag_external_manual_override_ok		# external override non-fatal for system. Only true for fixed wing

bool flag_control_manual_enabled		# true if manual input is mixed in
bool flag_control_auto_enabled			# true if onboard autopilot should act
bool flag_control_offboard_enabled		# true if offboard control should be used
bool flag_control_rates_enabled			# true if rates are stabilized
bool flag_control_attitude_enabled		# true if attitude stabilization is mixed in
bool flag_control_acceleration_enabled		# true if acceleration is controlled
bool flag_control_velocity_enabled		# true if horizontal velocity (implies direction) is controlled
bool flag_control_position_enabled		# true if position is controlled
bool flag_control_altitude_enabled		# true if altitude is controlled
bool flag_control_climb_rate_enabled		# true if climb rate is controlled
bool flag_control_termination_enabled		# true if flighttermination is enabled
