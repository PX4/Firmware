uint64 timestamp			# time since system start (microseconds)
uint64 timestamp_sample			# the timestamp of the raw data (microseconds)

float32[2] vel_body			# velocity obtained from gyro-compensated and distance-scaled optical flow raw measurements in body frame(m/s)
float32[2] vel_ne			# same as vel_body but in local frame (m/s)
float32[2] flow_uncompensated_integral	# integrated optical flow measurement (rad)
float32[2] flow_compensated_integral	# integrated optical flow measurement compensated for angular motion (rad)
float32[3] gyro_rate_integral		# gyro measurement integrated to flow rate and synchronized with flow measurements (rad)
