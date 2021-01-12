uint64 timestamp		# time since system start (microseconds)
uint8 len			# length of data
uint8 flags         		# LSB: 1=fragmented
uint8[182] data		# data to write to GPS device (RTCM message)

uint8 ORB_QUEUE_LENGTH = 8
