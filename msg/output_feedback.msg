uint64 timestamp         # time since system start (microseconds)

uint8 MAX_ACTUATORS = 8  # Max actuators available in a single topic publication

uint16[8] function       # Actuator function being controlled
float32[8] feedback      # Feedback of setpoint value that was actually applied
uint8[8] status          # Status flags

uint8 STATUS_UNKNOWN = 0
uint8 STATUS_STANDBY = 1
uint8 STATUS_ARMED = 2
uint8 STATUS_OPERATING = 4
uint8 STATUS_ERROR = 8
