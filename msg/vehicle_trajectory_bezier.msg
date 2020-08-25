# Vehicle Waypoints Trajectory description. See also MAVLink MAV_TRAJECTORY_REPRESENTATION msg
# The topic vehicle_trajectory_bezier is used to send a smooth flight path from the
# companion computer / avoidance module to the position controller.

uint64 timestamp		# time since system start (microseconds)

uint8 POINT_0 = 0
uint8 POINT_1 = 1
uint8 POINT_2 = 2
uint8 POINT_3 = 3
uint8 POINT_4 = 4

uint8 NUMBER_POINTS = 5

trajectory_bezier[5] control_points
uint8 bezier_order

# TOPICS vehicle_trajectory_bezier
