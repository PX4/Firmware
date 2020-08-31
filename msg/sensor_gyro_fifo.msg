uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id          # unique device ID for the sensor that does not change between power cycles

float32 dt                # delta time between samples (microseconds)
float32 scale

uint8 samples             # number of valid samples

int16[32] x               # angular velocity in the NED X board axis in rad/s
int16[32] y               # angular velocity in the NED Y board axis in rad/s
int16[32] z               # angular velocity in the NED Z board axis in rad/s

uint8 rotation            # Direction the sensor faces (see Rotation enum)
