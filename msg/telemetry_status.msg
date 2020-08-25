uint8 LINK_TYPE_GENERIC = 0
uint8 LINK_TYPE_UBIQUITY_BULLET = 1
uint8 LINK_TYPE_WIRE = 2
uint8 LINK_TYPE_USB = 3
uint8 LINK_TYPE_IRIDIUM	= 4

# MAV_COMPONENT (fill in as needed)
uint8 COMPONENT_ID_ALL		= 0
uint8 COMPONENT_ID_AUTOPILOT1	= 1
uint8 COMPONENT_ID_CAMERA	= 100
uint8 COMPONENT_ID_OBSTACLE_AVOIDANCE = 196

# MAV_TYPE (fill in as needed)
uint8 MAV_TYPE_GENERIC = 0
uint8 MAV_TYPE_GCS = 6
uint8 MAV_TYPE_ONBOARD_CONTROLLER = 18

# MAV_STATE
uint8 MAV_STATE_UNINIT = 0
uint8 MAV_STATE_BOOT = 1
uint8 MAV_STATE_CALIBRATING = 2
uint8 MAV_STATE_STANDBY = 3
uint8 MAV_STATE_ACTIVE = 4
uint8 MAV_STATE_CRITICAL = 5
uint8 MAV_STATE_EMERGENCY = 6
uint8 MAV_STATE_POWEROFF = 7
uint8 MAV_STATE_FLIGHT_TERMINATION = 8

uint64 timestamp			# time since system start (microseconds)

uint64 heartbeat_time			# Time of last received heartbeat from remote system

uint8 remote_system_id			# system id of the remote system (Mavlink header sys_id)
uint8 remote_component_id		# component id of the remote system (Mavlink header comp_id)
uint8 remote_type			# remote system status flag (Mavlink MAV_TYPE)
uint8 remote_system_status		# remote system status flag (Mavlink MAV_STATE)

uint8 type				#  type of the radio hardware (LINK_TYPE_*)

uint8 mode

bool flow_control
bool forwarding
bool mavlink_v2
bool ftp

uint8 streams

float32 data_rate

float32 rate_multiplier

float32 rate_rx

float32 rate_tx
float32 rate_txerr

uint8 ORB_QUEUE_LENGTH = 3
