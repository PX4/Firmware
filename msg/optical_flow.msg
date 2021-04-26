# Optical flow in XYZ body frame in SI units.
# http://en.wikipedia.org/wiki/International_System_of_Units

uint64 timestamp		# time since system start (microseconds)

uint8 sensor_id			# id of the sensor emitting the flow value
float32 pixel_flow_x_integral	# accumulated optical flow in radians where a positive value is produced by a RH rotation about the X body axis
float32 pixel_flow_y_integral	# accumulated optical flow in radians where a positive value is produced by a RH rotation about the Y body axis
float32 gyro_x_rate_integral	# accumulated gyro value in radians where a positive value is produced by a RH rotation about the X body axis. Set to NaN if flow sensor does not have 3-axis gyro data.
float32 gyro_y_rate_integral	# accumulated gyro value in radians where a positive value is produced by a RH rotation about the Y body axis. Set to NaN if flow sensor does not have 3-axis gyro data.
float32 gyro_z_rate_integral	# accumulated gyro value in radians where a positive value is produced by a RH rotation about the Z body axis. Set to NaN if flow sensor does not have 3-axis gyro data.
float32 ground_distance_m	# Altitude / distance to ground in meters
uint32 integration_timespan	# accumulation timespan in microseconds
uint32 time_since_last_sonar_update	# time since last sonar update in microseconds
uint16 frame_count_since_last_readout	# number of accumulated frames in timespan
int16 gyro_temperature	# Temperature * 100 in centi-degrees Celsius
uint8 quality	# Average of quality of accumulated frames, 0: bad quality, 255: maximum quality

float32 max_flow_rate # Magnitude of maximum angular which the optical flow sensor can measure reliably
float32 min_ground_distance # Minimum distance from ground at which the optical flow sensor operates reliably
float32 max_ground_distance # Maximum distance from ground at which the optical flow sensor operates reliably


uint8 MODE_UNKNOWN        = 0
uint8 MODE_BRIGHT         = 1
uint8 MODE_LOWLIGHT       = 2
uint8 MODE_SUPER_LOWLIGHT = 3

uint8 mode
