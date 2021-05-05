# this file is only used in the position_setpoint triple as a dependency

uint64 timestamp                 # time since system start (microseconds)

uint8 SETPOINT_TYPE_POSITION = 0 # position setpoint
uint8 SETPOINT_TYPE_LOITER   = 1 # loiter setpoint
uint8 SETPOINT_TYPE_TAKEOFF  = 2 # takeoff setpoint
uint8 SETPOINT_TYPE_LAND     = 3 # land setpoint, altitude must be ignored, descend until landing
uint8 SETPOINT_TYPE_IDLE     = 4 # do nothing, switch off motors or keep at idle speed (MC)

uint8 type                       # setpoint type to adjust behavior of position controller

bool valid                       # true if setpoint is valid

float64 lat                      # latitude, in deg
float64 lon                      # longitude, in deg
float32 alt                      # altitude AMSL, in m

float32 yaw                      # yaw (only for multirotors), in rad [-PI..PI), NaN = hold current yaw
bool yaw_valid                   # true if yaw setpoint valid

float32 loiter_radius            # loiter radius (only for fixed wing), in m
int8 loiter_direction            # loiter direction: 1 = CW, -1 = CCW

float32 acceptance_radius        # navigation acceptance_radius if we're doing waypoint navigation

float32 cruising_speed           # the generally desired cruising speed (not a hard constraint)
float32 cruising_throttle        # the generally desired cruising throttle (not a hard constraint)

bool disable_weather_vane        # VTOL: disable (in auto mode) the weather vane feature that turns the nose into the wind
