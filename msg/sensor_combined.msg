# Sensor readings in SI-unit form.
# These fields are scaled and offset-compensated where possible and do not
# change with board revisions and sensor updates.

uint64 timestamp				# time since system start (microseconds)

int32 RELATIVE_TIMESTAMP_INVALID = 2147483647	# (0x7fffffff) If one of the relative timestamps is set to this value, it means the associated sensor values are invalid

# gyro timstamp is equal to the timestamp of the message
float32[3] gyro_rad			# average angular rate measured in the XYZ body frame in rad/s over the last gyro sampling period
uint32 gyro_integral_dt		# gyro measurement sampling period in us

int32 accelerometer_timestamp_relative	# timestamp + accelerometer_timestamp_relative = Accelerometer timestamp
float32[3] accelerometer_m_s2		# average value acceleration measured in the XYZ body frame in m/s/s over the last accelerometer sampling period
uint32 accelerometer_integral_dt	# accelerometer measurement sampling period in us

uint8 CLIPPING_X = 1
uint8 CLIPPING_Y = 2
uint8 CLIPPING_Z = 4
uint8 accelerometer_clipping            # bitfield indicating if there was any accelerometer clipping (per axis) during the sampling period
