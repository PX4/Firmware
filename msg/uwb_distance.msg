# UWB distance contains the distance information measured by an ultra-wideband positioning system,
# such as Pozyx or NXP Rddrone.

uint64 	timestamp		# time since system start (microseconds)
uint8 	status			# status feedback
uint16 	counter			# Number of Ranges since last Start of Ranging
uint8 	time_offset		#  in Milie Seconds  //Populate with actual time of Range?
float32 yaw_offset		# Yaw offset in degrees
uint16[9] anchor_distance	# distance in cm
int32[3] position		# position of the drone (x,y,z) in regards to (to be determined)


