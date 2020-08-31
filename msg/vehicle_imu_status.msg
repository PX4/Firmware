uint64 timestamp          # time since system start (microseconds)

uint32 accel_device_id    # unique device ID for the sensor that does not change between power cycles
uint32 gyro_device_id    # unique device ID for the sensor that does not change between power cycles

uint32[3] accel_clipping        # clipping per axis

uint32 accel_error_count
uint32 gyro_error_count

uint16 accel_rate_hz
uint16 gyro_rate_hz

float32 accel_vibration_metric  # high frequency vibration level in the IMU delta velocity data (m/s)
float32 gyro_vibration_metric  # high frequency vibration level in the IMU delta velocity data (m/s)
float32 gyro_coning_vibration  # Level of coning vibration in the IMU delta angles (rad^2)
