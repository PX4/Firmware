uint64 timestamp          # time since system start (microseconds)


# COMPONENT (fill in as needed)
uint8 COMP_ID_ALL                      = 0
uint8 COMP_ID_AUTOPILOT1               = 1
uint8 COMP_ID_TELEMETRY_RADIO          = 68
uint8 COMP_ID_CAMERA                   = 100
uint8 COMP_ID_GIMBAL                   = 154
uint8 COMP_ID_LOG                      = 155
uint8 COMP_ID_ADSB                     = 156
uint8 COMP_ID_OSD                      = 157
uint8 COMP_ID_PERIPHERAL               = 158
uint8 COMP_ID_FLARM                    = 160
uint8 COMP_ID_MISSIONPLANNER           = 190
uint8 COMP_ID_OBSTACLE_AVOIDANCE       = 196
uint8 COMP_ID_VISUAL_INERTIAL_ODOMETRY = 197
uint8 COMP_ID_PAIRING_MANAGER          = 198
uint8 COMP_ID_UDP_BRIDGE               = 240
uint8 COMP_ID_UART_BRIDGE              = 241
uint8 COMP_ID_TUNNEL_NODE              = 242

uint8 system_id           # system id of the remote system (Mavlink header sys_id)
uint8 component_id        # component id of the remote system (Mavlink header comp_id)


# TYPE (fill in as needed)
uint8 TYPE_GENERIC                     = 0
uint8 TYPE_ANTENNA_TRACKER             = 5
uint8 TYPE_GCS                         = 6
uint8 TYPE_ONBOARD_CONTROLLER          = 18
uint8 TYPE_GIMBAL                      = 26
uint8 TYPE_ADSB                        = 27
uint8 TYPE_CAMERA                      = 30
uint8 TYPE_CHARGING_STATION            = 31

uint8 type


# STATE
uint8 STATE_UNINIT                     = 0
uint8 STATE_BOOT                       = 1
uint8 STATE_CALIBRATING                = 2
uint8 STATE_STANDBY                    = 3
uint8 STATE_ACTIVE                     = 4
uint8 STATE_CRITICAL                   = 5
uint8 STATE_EMERGENCY                  = 6
uint8 STATE_POWEROFF                   = 7
uint8 STATE_FLIGHT_TERMINATION         = 8

uint8 state
