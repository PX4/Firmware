# UWB report message contains the grid information measured by an ultra-wideband positioning system,
# such as Pozyx or NXP Rddrone.

uint64 	timestamp	# time since system start (microseconds)
uint16 	initator_time	# time to synchronize clocks
uint8 flag	 	# flags
uint8[16] grid_uuid	#
uint8 anchor_nr	 	# nr of Anchors

# GPS:
bool global_pos_valid  # True iff the values origin_lat, origin_lon, and origin_alt are all valid
float32[4] gps
float32 lat  # Latitude of the point (0, 0, 0) of the UWB system
float32 lon  # Longitude of the point (0, 0, 0) of the UWB system
float32 alt  # Altitude of the point (0, 0, 0) of the UWB system
float32 yaw		# Yaw offset in degrees

#Grid DATA
#Coordinates start at Anchor 0 with X defined by Anchor 1
float32[3] target_pos 	#Target, mostly landing Target x,y,z
float32[3] anchor_pos_0 #Anchor 0
float32[3] anchor_pos_1 #better way?
float32[3] anchor_pos_2 #Position of every anchor
float32[3] anchor_pos_3 #Position of every anchor
float32[3] anchor_pos_4 #Position of every anchor
float32[3] anchor_pos_5 #Position of every anchor
float32[3] anchor_pos_6 #Position of every anchor
float32[3] anchor_pos_7 #Position of every anchor
float32[3] anchor_pos_8 #Position of every anchor



# TODO: We could add the following fields. They would allow us to calculate the vehicle's global position by simply
#  adding the offset (measured by the UWB system) to the origin lat/lon/alt.
#  However, this functionality is not yet needed or used anywhere, so it is omitted for now to save space.

# bool global_pos_valid  # True iff the values origin_lat, origin_lon, and origin_alt are all valid
# float32 origin_lat  # Latitude of the point (0, 0, 0) of the UWB system
# float32 origin_lon  # Longitude of the point (0, 0, 0) of the UWB system
# float32 origin_alt  # Altitude of the point (0, 0, 0) of the UWB system
