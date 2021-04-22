# UWB distance contains the distance information measured by an ultra-wideband positioning system,
# such as Pozyx or NXP Rddrone.

uint64 	timestamp		# time since system start (microseconds)
uint32 	counter			# Number of Ranges since last Start of Ranging
uint16 	status			# status feedback
uint16[9] anchor_distance	# distance in cm
uint8 	time_offset		#  in Milie Seconds  //Populate with actual time of Range?
# GPS:
#GPS position of target_pos of the UWB system
float32[4] gps_data #Latitude, Longitude, Altitude, and Yaw Offset to True North

float32[3] position		# Position of the Landing point in relation to the Drone (x,y,z in Meters NED)
				# X/north position of target, relative to vehicle (navigation frame) [meters]
				# Y/east position of target, relative to vehicle (navigation frame) [meters]
				# Z/down position of target, relative to vehicle (navigation frame) [meters]


