# Local position setpoint in NED frame
# setting something to NaN means the state should not be controlled

uint64 timestamp	# time since system start (microseconds)

float32[3] position     # in meters NED
float32[3] velocity     # in meters/sec
float32[3] acceleration # in meters/sec^2
float32[3] jerk         # in meters/sec^3 (logging only)

float32 yaw             # in radians NED -PI..+PI
float32 yawspeed        # in radians/sec
