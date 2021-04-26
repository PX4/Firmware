uint64 timestamp                # time since system start (microseconds)

uint32 accel_device_id          # unique device ID for the sensor that does not change between power cycles
uint32 gyro_device_id           # unique device ID for the sensor that does not change between power cycles

uint32[3] accel_clipping        # total clipping per axis

uint32 accel_error_count
uint32 gyro_error_count

float32 accel_rate_hz
float32 gyro_rate_hz

float32 accel_raw_rate_hz       # full raw sensor sample rate (Hz)
float32 gyro_raw_rate_hz        # full raw sensor sample rate (Hz)

float32 accel_vibration_metric  # high frequency vibration level in the IMU delta velocity data (m/s)
float32 gyro_vibration_metric   # high frequency vibration level in the IMU delta velocity data (m/s)
float32 gyro_coning_vibration   # Level of coning vibration in the IMU delta angles (rad^2)

float32[3] mean_accel           # average accelerometer readings since last publication
float32[3] mean_gyro            # average gyroscope readings since last publication

float32 temperature_accel
float32 temperature_gyro
