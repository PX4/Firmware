uint64 timestamp	# time since system start (microseconds)
uint32 icao_address 	# ICAO address
float64 lat 		# Latitude, expressed as degrees
float64 lon 		# Longitude, expressed as degrees
uint8 altitude_type	# Type from ADSB_ALTITUDE_TYPE enum
float32 altitude 	# Altitude(ASL) in meters
float32 heading 	# Course over ground in radians, -pi to +pi, 0 is north
float32 hor_velocity	# The horizontal velocity in m/s
float32 ver_velocity 	# The vertical velocity in m/s, positive is up
char[9] callsign	# The callsign, 8+null
uint8 emitter_type 	# Type from ADSB_EMITTER_TYPE enum
uint8 tslc 		# Time since last communication in seconds
uint16 flags 		# Flags to indicate various statuses including valid data fields
uint16 squawk 		# Squawk code
uint8[18] uas_id	# Unique UAS ID

# ADSB flags
uint16 PX4_ADSB_FLAGS_VALID_COORDS = 1
uint16 PX4_ADSB_FLAGS_VALID_ALTITUDE = 2
uint16 PX4_ADSB_FLAGS_VALID_HEADING = 4
uint16 PX4_ADSB_FLAGS_VALID_VELOCITY = 8
uint16 PX4_ADSB_FLAGS_VALID_CALLSIGN = 16
uint16 PX4_ADSB_FLAGS_VALID_SQUAWK = 32
uint16 PX4_ADSB_FLAGS_RETRANSLATE = 256

#ADSB Emitter Data:
#from mavlink/v2.0/common/common.h
uint16 ADSB_EMITTER_TYPE_NO_INFO=0
uint16 ADSB_EMITTER_TYPE_LIGHT=1
uint16 ADSB_EMITTER_TYPE_SMALL=2
uint16 ADSB_EMITTER_TYPE_LARGE=3
uint16 ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE=4
uint16 ADSB_EMITTER_TYPE_HEAVY=5
uint16 ADSB_EMITTER_TYPE_HIGHLY_MANUV=6
uint16 ADSB_EMITTER_TYPE_ROTOCRAFT=7
uint16 ADSB_EMITTER_TYPE_UNASSIGNED=8
uint16 ADSB_EMITTER_TYPE_GLIDER=9
uint16 ADSB_EMITTER_TYPE_LIGHTER_AIR=10
uint16 ADSB_EMITTER_TYPE_PARACHUTE=11
uint16 ADSB_EMITTER_TYPE_ULTRA_LIGHT=12
uint16 ADSB_EMITTER_TYPE_UNASSIGNED2=13
uint16 ADSB_EMITTER_TYPE_UAV=14
uint16 ADSB_EMITTER_TYPE_SPACE=15
uint16 ADSB_EMITTER_TYPE_UNASSGINED3=16
uint16 ADSB_EMITTER_TYPE_EMERGENCY_SURFACE=17
uint16 ADSB_EMITTER_TYPE_SERVICE_SURFACE=18
uint16 ADSB_EMITTER_TYPE_POINT_OBSTACLE=19
uint16 ADSB_EMITTER_TYPE_ENUM_END=20

uint8 ORB_QUEUE_LENGTH = 8
