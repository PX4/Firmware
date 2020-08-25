#
# Sensor readings and in-run biases in SI-unit form. Sensor readings are compensated for static offsets,
# scale errors, in-run bias and thermal drift (if thermal compensation is enabled and available).
#

uint64 timestamp		# time since system start (microseconds)

# In-run bias estimates (subtract from uncorrected data)

uint32 gyro_device_id		# unique device ID for the sensor that does not change between power cycles
float32[3] gyro_bias		# gyroscope in-run bias in body frame (rad/s)

uint32 accel_device_id		# unique device ID for the sensor that does not change between power cycles
float32[3] accel_bias		# accelerometer in-run bias in body frame (m/s^2)

uint32 mag_device_id		# unique device ID for the sensor that does not change between power cycles
float32[3] mag_bias		# magnetometer in-run bias in body frame (Gauss)
