uint64 timestamp				# time since system start (microseconds)

uint32 instance_count		# Instance count of this mission. Increments monotonically whenever the mission is modified

uint64 mission_timestamp
uint8 mission_dataman_id	# default 0, there are two offboard storage places in the dataman: 0 or 1

int32 seq_reached		# Sequence of the mission item which has been reached, default -1
uint16 seq_current		# Sequence of the current mission item
uint16 seq_total		# Total number of mission items

bool active                    # true if mission is currently active (executing)
bool valid			# true if mission is valid
bool warning			# true if mission is valid, but has potentially problematic items leading to safety warnings
bool finished			# true if mission has been completed
bool failure			# true if the mission cannot continue or be completed for some reason

bool item_do_jump_changed	# true if the number of do jumps remaining has changed
uint16 item_changed_index	# indicate which item has changed
uint16 item_do_jump_remaining	# set to the number of do jumps remaining for that item

uint8 MISSION_EXECUTION_MODE_NORMAL = 0	# Execute the mission according to the planned items
uint8 MISSION_EXECUTION_MODE_REVERSE = 1	# Execute the mission in reverse order, ignoring commands and converting all waypoints to normal ones
uint8 MISSION_EXECUTION_MODE_FAST_FORWARD = 2	# Execute the mission as fast as possible, for example converting loiter waypoints to normal ones

uint8 execution_mode	# indicates the mode in which the mission is executed
