# IMU readings in SI-unit form.

uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample

uint32 accel_device_id          # Accelerometer unique device ID for the sensor that does not change between power cycles
uint32 gyro_device_id           # Gyroscope unique device ID for the sensor that does not change between power cycles

float32[3] delta_angle          # delta angle in the NED board axis in rad/s over the integration time frame (dt)
float32[3] delta_velocity       # delta velocity in the NED board axis in m/s over the integration time frame (dt)
uint16 delta_angle_dt           # integration period in us
uint16 delta_velocity_dt        # integration period in us

uint8 CLIPPING_X = 1
uint8 CLIPPING_Y = 2
uint8 CLIPPING_Z = 4
uint8 delta_velocity_clipping   # bitfield indicating if there was any accelerometer clipping (per axis) during the integration time frame
