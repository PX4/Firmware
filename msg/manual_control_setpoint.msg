uint64 timestamp			# time since system start (microseconds)

uint64 timestamp_sample                # the timestamp of the raw data (microseconds)

uint8 SOURCE_RC = 1			# radio control
uint8 SOURCE_MAVLINK_0 = 2		# mavlink instance 0
uint8 SOURCE_MAVLINK_1 = 3		# mavlink instance 1
uint8 SOURCE_MAVLINK_2 = 4		# mavlink instance 2
uint8 SOURCE_MAVLINK_3 = 5		# mavlink instance 4

uint8 data_source		 # where this input is coming from

# Any of the channels may not be available and be set to NaN
# to indicate that it does not contain valid data.

# Stick positions [-1,1]
# on a common RC mode 1/2/3/4 remote/joystick the stick deflection: -1 is down/left, 1 is up/right
# Positive directions are generally used for:
# x - forward, negative pitch,  nose down
# y - right,   positive roll,   right side down
# z - up,      positive thrust, -1 is 0% thrust +1 is 100% thrust
# r - righthand rotation around vertical downwards axis (= clockwise yaw rotation when seen from above)
# The definition is analogous to the MANUAL_CONTROL mavlink message's x, y, z, r fields.
# Note: QGC sends z in range [0,1000] - [0,1]. The MAVLink input conversion [0,1] to [-1,1] is at the moment kept backwards compatible.
float32[4] xyzr

float32 flaps			 # flap position

float32 aux1			 # default function: camera yaw / azimuth
float32 aux2			 # default function: camera pitch / tilt
float32 aux3			 # default function: camera trigger
float32 aux4			 # default function: camera roll
float32 aux5			 # default function: payload drop
float32 aux6

# DEPRECATED: float32 x
# DEPRECATED: float32 y
# DEPRECATED: float32 z
# DEPRECATED: float32 r
