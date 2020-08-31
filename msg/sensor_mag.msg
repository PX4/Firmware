uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id          # unique device ID for the sensor that does not change between power cycles

float32 x                 # magnetic field in the NED X board axis in Gauss
float32 y                 # magnetic field in the NED Y board axis in Gauss
float32 z                 # magnetic field in the NED Z board axis in Gauss

float32 temperature       # temperature in degrees celsius

uint32 error_count

bool is_external	# if true the mag is external (i.e. not built into the board)

uint8 ORB_QUEUE_LENGTH = 3
