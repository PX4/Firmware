uint8 LINK_TYPE_GENERIC = 0
uint8 LINK_TYPE_UBIQUITY_BULLET = 1
uint8 LINK_TYPE_WIRE = 2
uint8 LINK_TYPE_USB = 3
uint8 LINK_TYPE_IRIDIUM	= 4

uint64 timestamp			# time since system start (microseconds)

uint8 type				#  type of the radio hardware (LINK_TYPE_*)

uint8 mode

bool flow_control
bool forwarding
bool mavlink_v2
bool ftp

uint8 streams

float32 data_rate                       # configured maximum data rate (Bytes/s)

float32 rate_multiplier

float32 tx_rate_avg                     # transmit rate average (Bytes/s)
float32 tx_error_rate_avg               # transmit error rate average (Bytes/s)
uint32 tx_message_count                 # total message sent count
uint32 tx_buffer_overruns               # number of TX buffer overruns

float32 rx_rate_avg                     # transmit rate average (Bytes/s)
uint32 rx_message_count                 # count of total messages received
uint32 rx_message_count_supported       # count of total messages received from supported systems and components (for loss statistics)
uint32 rx_message_lost_count
uint32 rx_buffer_overruns               # number of RX buffer overruns
uint32 rx_parse_errors                  # number of parse errors
uint32 rx_packet_drop_count             # number of packet drops
float32 rx_message_lost_rate


uint64 HEARTBEAT_TIMEOUT_US = 1500000       # Heartbeat timeout 1.5 seconds

# Heartbeats per type
bool heartbeat_type_antenna_tracker         # MAV_TYPE_ANTENNA_TRACKER
bool heartbeat_type_gcs                     # MAV_TYPE_GCS
bool heartbeat_type_onboard_controller      # MAV_TYPE_ONBOARD_CONTROLLER
bool heartbeat_type_gimbal                  # MAV_TYPE_GIMBAL
bool heartbeat_type_adsb                    # MAV_TYPE_ADSB
bool heartbeat_type_camera                  # MAV_TYPE_CAMERA

# Heartbeats per component
bool heartbeat_component_telemetry_radio    # MAV_COMP_ID_TELEMETRY_RADIO
bool heartbeat_component_log                # MAV_COMP_ID_LOG
bool heartbeat_component_osd                # MAV_COMP_ID_OSD
bool heartbeat_component_obstacle_avoidance # MAV_COMP_ID_OBSTACLE_AVOIDANCE
bool heartbeat_component_vio                # MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
bool heartbeat_component_pairing_manager    # MAV_COMP_ID_PAIRING_MANAGER
bool heartbeat_component_udp_bridge         # MAV_COMP_ID_UDP_BRIDGE
bool heartbeat_component_uart_bridge        # MAV_COMP_ID_UART_BRIDGE

# Misc component health
bool avoidance_system_healthy
