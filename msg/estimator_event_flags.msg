uint64 timestamp                        # time since system start (microseconds)
uint64 timestamp_sample                 # the timestamp of the raw data (microseconds)

# information events
uint32 information_event_changes        # number of information event changes
bool gps_checks_passed                  #  0 - true when gps quality checks are passing passed
bool reset_vel_to_gps                   #  1 - true when the velocity states are reset to the gps measurement
bool reset_vel_to_flow                  #  2 - true when the velocity states are reset using the optical flow measurement
bool reset_vel_to_vision                #  3 - true when the velocity states are reset to the vision system measurement
bool reset_vel_to_zero                  #  4 - true when the velocity states are reset to zero
bool reset_pos_to_last_known            #  5 - true when the position states are reset to the last known position
bool reset_pos_to_gps                   #  6 - true when the position states are reset to the gps measurement
bool reset_pos_to_vision                #  7 - true when the position states are reset to the vision system measurement
bool starting_gps_fusion                #  8 - true when the filter starts using gps measurements to correct the state estimates
bool starting_vision_pos_fusion         #  9 - true when the filter starts using vision system position measurements to correct the state estimates
bool starting_vision_vel_fusion         # 10 - true when the filter starts using vision system velocity measurements to correct the state estimates
bool starting_vision_yaw_fusion         # 11 - true when the filter starts using vision system yaw  measurements to correct the state estimates
bool yaw_aligned_to_imu_gps             # 12 - true when the filter resets the yaw to an estimate derived from IMU and GPS data

# warning events
uint32 warning_event_changes            # number of warning event changes
bool gps_quality_poor                   #  0 - true when the gps is failing quality checks
bool gps_fusion_timout                  #  1 - true when the gps data has not been used to correct the state estimates for a significant time period
bool gps_data_stopped                   #  2 - true when the gps data has stopped for a significant time period
bool gps_data_stopped_using_alternate   #  3 - true when the gps data has stopped for a significant time period but the filter is able to use other sources of data to maintain navigation
bool height_sensor_timeout              #  4 - true when the height sensor has not been used to correct the state estimates for a significant time period
bool stopping_navigation                #  5 - true when the filter has insufficient data to estimate velocity and position and is falling back to an attitude, height and height rate mode of operation
bool invalid_accel_bias_cov_reset       #  6 - true when the filter has detected bad acceerometer bias state esitmstes and has reset the corresponding covariance matrix elements
bool bad_yaw_using_gps_course           #  7 - true when the fiter has detected an invalid yaw esitmate and has reset the yaw angle to the GPS ground course
bool stopping_mag_use                   #  8 - true when the filter has detected bad magnetometer data and is stopping further use of the magnetomer data
bool vision_data_stopped                #  9 - true when the vision system data has stopped for a significant time period
bool emergency_yaw_reset_mag_stopped    # 10 - true when the filter has detected bad magnetometer data, has reset the yaw to anothter source of data and has stopped further use of the magnetomer data
