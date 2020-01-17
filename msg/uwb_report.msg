# UWB report message contains the coordinates of the vehicle measured by an ultra-wideband positioning system,
# such as Pozyx or NXP Rddrone.

uint64 timestamp					# time since system start (microseconds)
# The coordinate frame of the UWB grid should be a Z-down, right-handed coordinate system.
# The yaw alignment is arbitrary, and stored in the yaw_offset field.
# If yaw_offset == 0, then the UWB grid is NED (North-East-Down)
float32 pos_x      # X position in the UWB grid frame, in meters
float32 pos_y      # Y position in the UWB grid frame, in meters
float32 pos_z      # Z position in the UWB grid frame, in meters

# The reason these values are kept in the uwb_report message is that we do not know the transform between the
# UWB coordinate system and the world coordinate system. Different UWB systems might have different mechanisms for
# this, so the UWB driver should handle it. Unless you
float32 target_pos_x  # X position of the vehicle relative to the UWB origin (usually the landing point), in NED frame, in meters
float32 target_pos_y  # Y position of the vehicle relative to the UWB origin (usually the landing point), in NED frame, in meters
float32 target_pos_z  # Z position of the vehicle relative to the UWB origin (usually the landing point), in NED frame, in meters

# TODO: We could add the following fields. They would allow us to calculate the vehicle's global position by simply
#  adding the offset (measured by the UWB system) to the origin lat/lon/alt.
#  However, this functionality is not yet needed or used anywhere, so it is omitted for now to save space.

# bool global_pos_valid  # True iff the values origin_lat, origin_lon, and origin_alt are all valid
# float32 origin_lat  # Latitude of the point (0, 0, 0) of the UWB system
# float32 origin_lon  # Longitude of the point (0, 0, 0) of the UWB system
# float32 origin_alt  # Altitude of the point (0, 0, 0) of the UWB system

int16 var_x
int16 var_y
int16 var_z

int16 cov_xy
int16 cov_xz
int16 cov_yz
