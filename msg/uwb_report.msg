uint64 timestamp					# time since system start (microseconds)
# The coordinate frame of the UWB grid should be a Z-down, right-handed coordinate system.
# The yaw alignment is arbitrary, and stored in the yaw_offset field.
# If yaw_offset == 0, then the UWB grid is NED (North-East-Down)
float32 pos_x      # X position in the UWB grid frame, in meters
float32 pos_y      # Y position in the UWB grid frame, in meters
float32 pos_z      # Z position in the UWB grid frame, in meters

#float32 yaw_offset  # Angle between the X axis of the UWB grid frame and true north. I.e., your compass heading if you face along the positive X axis

int16 var_x
int16 var_y
int16 var_z

int16 cov_xy
int16 cov_xz
int16 cov_yz
