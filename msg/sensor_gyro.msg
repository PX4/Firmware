uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id          # unique device ID for the sensor that does not change between power cycles

float32 x                 # angular velocity in the NED X board axis in rad/s
float32 y                 # angular velocity in the NED Y board axis in rad/s
float32 z                 # angular velocity in the NED Z board axis in rad/s

float32 temperature       # temperature in degrees celsius

uint32 error_count

uint8 ORB_QUEUE_LENGTH = 8
