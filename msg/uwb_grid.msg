# UWB report message contains the grid information measured by an ultra-wideband positioning system,
# such as Pozyx or NXP Rddrone.

uint64 	timestamp	# time since system start (microseconds)
uint16 	initator_time	# time to synchronize clocks
uint8 flag	 	# flag, bit 7: Grid is moving yes / no
uint8[16] grid_uuid	# UUID of Anchor 0 of the Grid
uint8 num_anchors	 	# Number of anchors

# GPS:
#Latitude of the point (0, 0, 0) of the UWB system
float32[4] gps #Latitude, Longitude, Altitude, and Yaw Offset to Truenorth
		#todo Improve accuracy to float64
		#yaw offset does not need this accuracy

#Grid DATA
#Coordinates start at Anchor 0 with X defined by Anchor 1
int32[3] target_pos 	#Target, mostly landing Target x,y,z
int32[3] anchor_pos_0 #Anchor 0
int32[3] anchor_pos_1 #better way?
int32[3] anchor_pos_2 #Position of every anchor
int32[3] anchor_pos_3 #Position of every anchor
int32[3] anchor_pos_4 #Position of every anchor
int32[3] anchor_pos_5 #Position of every anchor
int32[3] anchor_pos_6 #Position of every anchor
int32[3] anchor_pos_7 #Position of every anchor
int32[3] anchor_pos_8 #Position of every anchor


