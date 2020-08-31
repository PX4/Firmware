# ORBIT_YAW_BEHAVIOUR
uint8 ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER = 0
uint8 ORBIT_YAW_BEHAVIOUR_HOLD_INITIAL_HEADING = 1
uint8 ORBIT_YAW_BEHAVIOUR_UNCONTROLLED = 2
uint8 ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE = 3
uint8 ORBIT_YAW_BEHAVIOUR_RC_CONTROLLED = 4

uint64 timestamp # time since system start (microseconds)
float32 radius   # Radius of the orbit circle. Positive values orbit clockwise, negative values orbit counter-clockwise. [m]
uint8 frame      # The coordinate system of the fields: x, y, z.
float64 x        # X coordinate of center point. Coordinate system depends on frame field: local = x position in meters * 1e4, global = latitude in degrees * 1e7.
float64 y        # Y coordinate of center point. Coordinate system depends on frame field: local = y position in meters * 1e4, global = latitude in degrees * 1e7.
float32 z        # Altitude of center point. Coordinate system depends on frame field.
uint8 yaw_behaviour
