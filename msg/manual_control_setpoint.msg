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
# The variable names follow the definition of the
# MANUAL_CONTROL mavlink message.
# The default range is from -1 to 1 (mavlink message -1000 to 1000)
# The range for the z variable is defined from 0 to 1. (The z field of
# the MANUAL_CONTROL mavlink message is defined from -1000 to 1000)

float32 x			 # stick position in x direction -1..1
				 # in general corresponds to forward/back motion or pitch of vehicle,
				 # in general a positive value means forward or negative pitch and
				 # a negative value means backward or positive pitch

float32 y			 # stick position in y direction -1..1
				 # in general corresponds to right/left motion or roll of vehicle,
				 # in general a positive value means right or positive roll and
				 # a negative value means left or negative roll

float32 z			 # throttle stick position 0..1
				 # in general corresponds to up/down motion or thrust of vehicle,
				 # in general the value corresponds to the demanded throttle by the user,
				 # if the input is used for setting the setpoint of a vertical position
				 # controller any value > 0.5 means up and any value < 0.5 means down

float32 r			 # yaw stick/twist position, -1..1
				 # in general corresponds to the righthand rotation around the vertical
				 # (downwards) axis of the vehicle

float32 flaps			 # flap position

float32 aux1			 # default function: camera yaw / azimuth
float32 aux2			 # default function: camera pitch / tilt
float32 aux3			 # default function: camera trigger
float32 aux4			 # default function: camera roll
float32 aux5			 # default function: payload drop
float32 aux6

