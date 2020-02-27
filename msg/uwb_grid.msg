# UWB report message contains the grid information measured by an ultra-wideband positioning system,
# such as Pozyx or NXP Rddrone.

uint64 	timestamp	# time since system start (microseconds)
uint8[16] grid_uuid	#
uint16 	time		# time to synchronize clocks
uint8 anchor_nr	 	#nr of Anchors

# GPS:
bool global_pos_valid  # True iff the values origin_lat, origin_lon, and origin_alt are all valid
float32 lat  # Latitude of the point (0, 0, 0) of the UWB system
float32 lon  # Longitude of the point (0, 0, 0) of the UWB system
float32 alt  # Altitude of the point (0, 0, 0) of the UWB system
float32 yaw		# Yaw offset in degrees

#Grid DATA
#Coordinates start at Anchor 0 with X defined by Anchor 1
float32[3] target_pos 	#Target, mostly landing Target x,y,z
float32[9] anchor_pos_x #Position of everyachor #2 dimensional array?
float32[9] anchor_pos_y #Position of everyachor #2 dimensional array?
float32[9] anchor_pos_z #Position of everyachor #2 dimensional array?


# TODO: We could add the following fields. They would allow us to calculate the vehicle's global position by simply
#  adding the offset (measured by the UWB system) to the origin lat/lon/alt.
#  However, this functionality is not yet needed or used anywhere, so it is omitted for now to save space.

# bool global_pos_valid  # True iff the values origin_lat, origin_lon, and origin_alt are all valid
# float32 origin_lat  # Latitude of the point (0, 0, 0) of the UWB system
# float32 origin_lon  # Longitude of the point (0, 0, 0) of the UWB system
# float32 origin_alt  # Altitude of the point (0, 0, 0) of the UWB system
