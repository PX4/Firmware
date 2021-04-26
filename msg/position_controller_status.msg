uint64 timestamp		# time since system start (microseconds)

float32 nav_roll
float32 nav_pitch
float32 nav_bearing 	# set to NAN if not valid

float32 target_bearing	# set to NAN if not valid
float32 xtrack_error	# set to NAN if not valid

float32 wp_dist

float32 acceptance_radius		# the optimal distance to a waypoint to switch to the next

float32 yaw_acceptance			# NaN if not set

float32 altitude_acceptance		# the optimal vertical distance to a waypoint to switch to the next

uint8 type
