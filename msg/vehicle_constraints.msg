# Local setpoint constraints in NED frame
# setting something to NaN means that no limit is provided

uint64 timestamp # time since system start (microseconds)

float32 yawspeed # in radians/sec
float32 speed_xy # in meters/sec
float32 speed_up # in meters/sec
float32 speed_down # in meters/sec
float32 tilt # in radians [0, PI]
float32 min_distance_to_ground # in meters
float32 max_distance_to_ground # in meters
bool want_takeoff # tell the controller to initiate takeoff when idling (ignored during flight)
bool reset_integral # tells controller to reset integrators e.g. since we know the vehicle is not in air
float32 minimum_thrust # tell controller what the minimum collective output thrust should be

uint32 flight_task
