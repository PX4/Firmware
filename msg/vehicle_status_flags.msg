# This is a struct used by the commander internally.

uint64 timestamp				# time since system start (microseconds)

bool condition_calibration_enabled
bool condition_system_sensors_initialized
bool condition_system_hotplug_timeout		# true if the hotplug sensor search is over
bool condition_system_returned_to_home
bool condition_auto_mission_available
bool condition_angular_velocity_valid
bool condition_attitude_valid
bool condition_local_altitude_valid
bool condition_local_position_valid		# set to true by the commander app if the quality of the local position estimate is good enough to use for navigation
bool condition_local_velocity_valid		# set to true by the commander app if the quality of the local horizontal velocity data is good enough to use for navigation
bool condition_global_position_valid		# set to true by the commander app if the quality of the global position estimate is good enough to use for navigation
bool condition_home_position_valid		# indicates a valid home position (a valid home position is not always a valid launch)
bool condition_airspeed_valid			# set to true if airspeed mesurements are valid
bool condition_power_input_valid                # set if input power is valid
bool condition_battery_healthy                # set if battery is available and not low
bool condition_escs_error		      # set to true if one or more ESCs reporting esc_status are offline
bool condition_escs_failure		      # set to true if one or more ESCs reporting esc_status has a failure

bool circuit_breaker_engaged_power_check
bool circuit_breaker_engaged_airspd_check
bool circuit_breaker_engaged_enginefailure_check
bool circuit_breaker_flight_termination_disabled
bool circuit_breaker_engaged_usb_check
bool circuit_breaker_engaged_posfailure_check	# set to true when the position valid checks have been disabled
bool circuit_breaker_vtol_fw_arming_check 	# set to true if for VTOLs arming in fixed-wing mode should be allowed

bool offboard_control_signal_found_once
bool offboard_control_signal_lost

bool rc_signal_found_once
bool rc_input_blocked                                # set if RC input should be ignored temporarily
bool rc_calibration_valid                            # set if RC calibration is valid
bool vtol_transition_failure                        # Set to true if vtol transition failed
bool usb_connected                                # status of the USB power supply
bool sd_card_detected_once                        # set to true if the SD card was detected

bool avoidance_system_required					  # Set to true if avoidance system is enabled via COM_OBS_AVOID parameter
bool avoidance_system_valid                       # Status of the obstacle avoidance system
