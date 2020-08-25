uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id          # unique device ID for the sensor that does not change between power cycles

uint64 error_count

float32[3] delta_angle    # delta angle in the NED board axis in rad/s over the integration time frame (dt)
uint16 dt                 # integration time (microseconds)
uint8 samples             # number of samples integrated
uint8 clip_count          # total clip count per integration period on any axis
