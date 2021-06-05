#
# Sensor corrections in SI-unit form for the voted sensor
#

uint64 timestamp		# time since system start (microseconds)

# Corrections for gyro angular rate outputs where corrected_rate = raw_rate * gyro_scale + gyro_offset
# Note the corrections are in the sensor frame and must be applied before the sensor data is rotated into body frame
uint32[4] gyro_device_ids
float32[4] gyro_temperature
float32[3] gyro_offset_0	# gyro 0 XYZ offsets in the sensor frame in rad/s
float32[3] gyro_offset_1	# gyro 1 XYZ offsets in the sensor frame in rad/s
float32[3] gyro_offset_2	# gyro 2 XYZ offsets in the sensor frame in rad/s
float32[3] gyro_offset_3	# gyro 3 XYZ offsets in the sensor frame in rad/s

# Corrections for acceleromter acceleration outputs where corrected_accel = raw_accel * accel_scale + accel_offset
# Note the corrections are in the sensor frame and must be applied before the sensor data is rotated into body frame
uint32[4] accel_device_ids
float32[4] accel_temperature
float32[3] accel_offset_0	# accelerometer 0 offsets in the FRD board frame XYZ-axis in m/s^s
float32[3] accel_offset_1	# accelerometer 1 offsets in the FRD board frame XYZ-axis in m/s^s
float32[3] accel_offset_2	# accelerometer 2 offsets in the FRD board frame XYZ-axis in m/s^s
float32[3] accel_offset_3	# accelerometer 3 offsets in the FRD board frame XYZ-axis in m/s^s

# Corrections for barometric pressure outputs where corrected_pressure = raw_pressure * pressure_scale + pressure_offset
# Note the corrections are in the sensor frame and must be applied before the sensor data is rotated into body frame
uint32[4] baro_device_ids
float32[4] baro_temperature
float32 baro_offset_0		# barometric pressure 0 offsets in the sensor frame in Pascals
float32 baro_offset_1		# barometric pressure 1 offsets in the sensor frame in Pascals
float32 baro_offset_2		# barometric pressure 2 offsets in the sensor frame in Pascals
float32 baro_offset_3		# barometric pressure 3 offsets in the sensor frame in Pascals
