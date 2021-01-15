# UWB distance contains the distance information measured by an ultra-wideband positioning system,
# such as Pozyx or NXP Rddrone.

uint64 	timestamp		# time since system start (microseconds)
uint16 	counter			# Number of Ranges since last Start of Ranging
uint16 	status			# status feedback
uint16[9] anchor_distance	# distance in cm
uint8 	time_offset		#  in Milie Seconds  //Populate with actual time of Range?
# GPS:
#GPS position of target_pos of the UWB system
float32[4] gps_data #Latitude, Longitude, Altitude, and Yaw Offset to Truenorth

int32[3] position		# position of the drone (x,y,z) in regards to (to be determined)


