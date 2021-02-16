# This message is used to notify the system about one or more parameter changes

uint64 timestamp		# time since system start (microseconds)

uint32 instance		# Instance count - constantly incrementing

uint32 get_count
uint32 set_count
uint32 find_count
uint32 export_count

uint16 active
uint16 changed
uint16 custom_default
