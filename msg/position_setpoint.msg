# this file is only used in the position_setpoint triple as a dependency

uint64 timestamp		# time since system start (microseconds)

uint8 SETPOINT_TYPE_POSITION=0	# position setpoint
uint8 SETPOINT_TYPE_VELOCITY=1	# velocity setpoint
uint8 SETPOINT_TYPE_LOITER=2	# loiter setpoint
uint8 SETPOINT_TYPE_TAKEOFF=3	# takeoff setpoint
uint8 SETPOINT_TYPE_LAND=4	# land setpoint, altitude must be ignored, descend until landing
uint8 SETPOINT_TYPE_IDLE=5	# do nothing, switch off motors or keep at idle speed (MC)
uint8 SETPOINT_TYPE_FOLLOW_TARGET=6  # setpoint in NED frame (x, y, z, vx, vy, vz) set by follow target

uint8 VELOCITY_FRAME_LOCAL_NED = 1 # MAV_FRAME_LOCAL_NED
uint8 VELOCITY_FRAME_BODY_NED = 8 # MAV_FRAME_BODY_NED

bool valid			# true if setpoint is valid
uint8 type			# setpoint type to adjust behavior of position controller

float32 vx			# local velocity setpoint in m/s in NED
float32 vy			# local velocity setpoint in m/s in NED
float32 vz			# local velocity setpoint in m/s in NED
bool velocity_valid		# true if local velocity setpoint valid
uint8 velocity_frame		# to set velocity setpoints in NED or body
bool alt_valid		# do not set for 3D position control. Set to true if you want z-position control while doing vx,vy velocity control.

float64 lat			# latitude, in deg
float64 lon			# longitude, in deg
float32 alt			# altitude AMSL, in m
float32 yaw			# yaw (only for multirotors), in rad [-PI..PI), NaN = hold current yaw
bool yaw_valid			# true if yaw setpoint valid

float32 yawspeed		# yawspeed (only for multirotors, in rad/s)
bool yawspeed_valid		# true if yawspeed setpoint valid

int8 landing_gear		# landing gear: see definition of the states in landing_gear.msg

float32 loiter_radius		# loiter radius (only for fixed wing), in m
int8 loiter_direction		# loiter direction: 1 = CW, -1 = CCW

float32 acceptance_radius   # navigation acceptance_radius if we're doing waypoint navigation

float32 cruising_speed		# the generally desired cruising speed (not a hard constraint)
float32 cruising_throttle	# the generally desired cruising throttle (not a hard constraint)

bool disable_weather_vane   # VTOL: disable (in auto mode) the weather vane feature that turns the nose into the wind
