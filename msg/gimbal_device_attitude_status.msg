uint64 timestamp						# time since system start (microseconds)

uint8 target_system
uint8 target_component
uint16 device_flags

uint16 DEVICE_FLAGS_RETRACT = 1
uint16 DEVICE_FLAGS_NEUTRAL = 2
uint16 DEVICE_FLAGS_ROLL_LOCK = 4
uint16 DEVICE_FLAGS_PITCH_LOCK = 8
uint16 DEVICE_FLAGS_YAW_LOCK = 16

float32[4] q
float32 angular_velocity_x
float32 angular_velocity_y
float32 angular_velocity_z

uint32 failure_flags
