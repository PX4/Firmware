uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id          # unique device ID for the sensor that does not change between power cycles

float32 x                 # acceleration in the FRD board frame X-axis in m/s^2
float32 y                 # acceleration in the FRD board frame Y-axis in m/s^2
float32 z                 # acceleration in the FRD board frame Z-axis in m/s^2

float32 temperature       # temperature in degrees Celsius

uint32 error_count

uint8[3] clip_counter     # clip count per axis in the sample period

uint8 samples             # number of raw samples that went into this message

uint8 ORB_QUEUE_LENGTH = 8
