uint64 timestamp          # time since system start (microseconds)

uint32 device_id          # unique device ID for the sensor that does not change between power cycles

uint64 error_count

float32 temperature

uint8 rotation

uint32[3] clipping        # clipping per axis

uint16 measure_rate_hz

float32 full_scale_range

float32 vibration_metric  # high frequency vibration level in the IMU delta angle data (rad)

float32 coning_vibration  # Level of coning vibration in the IMU delta angles (rad^2)
