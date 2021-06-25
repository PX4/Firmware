# MAV_TUNNEL_PAYLOAD_TYPE enum

uint8 MAV_TUNNEL_PAYLOAD_TYPE_UNKNOWN = 0                # Encoding of payload unknown
uint8 MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED0 = 200    # Registered for STorM32 gimbal controller
uint8 MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED1 = 201    # Registered for STorM32 gimbal controller
uint8 MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED2 = 202    # Registered for STorM32 gimbal controller
uint8 MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED3 = 203    # Registered for STorM32 gimbal controller
uint8 MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED4 = 204    # Registered for STorM32 gimbal controller
uint8 MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED5 = 205    # Registered for STorM32 gimbal controller
uint8 MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED6 = 206    # Registered for STorM32 gimbal controller
uint8 MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED7 = 207    # Registered for STorM32 gimbal controller
uint8 MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED8 = 208    # Registered for STorM32 gimbal controller
uint8 MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED9 = 209    # Registered for STorM32 gimbal controller

uint64 timestamp	     # Time since system start (microseconds)
uint16 payload_type      # A code that identifies the content of the payload (0 for unknown, which is the default). If this code is less than 32768, it is a 'registered' payload type and the corresponding code should be added to the MAV_TUNNEL_PAYLOAD_TYPE enum. Software creators can register blocks of types as needed. Codes greater than 32767 are considered local experiments and should not be checked in to any widely distributed codebase.
uint8 target_system      # System ID (can be 0 for broadcast, but this is discouraged)
uint8 target_component   # Component ID (can be 0 for broadcast, but this is discouraged)
uint8 payload_length     # Length of the data transported in payload
uint8[128] payload       # Data itself
