uint64 timestamp		# time since system start (microseconds)
uint8 SAT_INFO_MAX_SATELLITES = 20

uint8 count			# Number of satellites visible to the receiver
uint8[20] svid	 		# Space vehicle ID [1..255], see scheme below
uint8[20] used			# 0: Satellite not used, 1: used for navigation
uint8[20] elevation		# Elevation (0: right on top of receiver, 90: on the horizon) of satellite
uint8[20] azimuth		# Direction of satellite, 0: 0 deg, 255: 360 deg.
uint8[20] snr			# dBHz, Signal to noise ratio of satellite C/N0, range 0..99, zero when not tracking this satellite.
uint8[20] prn                   # Satellite PRN code assignment, (psuedorandom number SBAS, valid codes are 120-144)
