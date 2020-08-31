uint64 timestamp	# time since system start (microseconds)

uint8 CELLULAR_STATUS_FLAG_UNKNOWN=0 # State unknown or not reportable
uint8 CELLULAR_STATUS_FLAG_FAILED=1 # velocity setpoint
uint8 CELLULAR_STATUS_FLAG_INITIALIZING=2 # Modem is being initialized
uint8 CELLULAR_STATUS_FLAG_LOCKED=3	# Modem is locked
uint8 CELLULAR_STATUS_FLAG_DISABLED=4	# Modem is not enabled and is powered down
uint8 CELLULAR_STATUS_FLAG_DISABLING=5	# Modem is currently transitioning to the CELLULAR_STATUS_FLAG_DISABLED state
uint8 CELLULAR_STATUS_FLAG_ENABLING=6 	# Modem is currently transitioning to the CELLULAR_STATUS_FLAG_ENABLED state
uint8 CELLULAR_STATUS_FLAG_ENABLED=7  # Modem is enabled and powered on but not registered with a network provider and not available for data connections
uint8 CELLULAR_STATUS_FLAG_SEARCHING=8  # Modem is searching for a network provider to register
uint8 CELLULAR_STATUS_FLAG_REGISTERED=9  # Modem is registered with a network provider, and data connections and messaging may be available for use
uint8 CELLULAR_STATUS_FLAG_DISCONNECTING=10  # Modem is disconnecting and deactivating the last active packet data bearer. This state will not be entered if more than one packet data bearer is active and one of the active bearers is deactivated
uint8 CELLULAR_STATUS_FLAG_CONNECTING=11  # Modem is activating and connecting the first packet data bearer. Subsequent bearer activations when another bearer is already active do not cause this state to be entered
uint8 CELLULAR_STATUS_FLAG_CONNECTED=12  # One or more packet data bearers is active and connected

uint8 CELLULAR_NETWORK_FAILED_REASON_NONE=0 # No error
uint8 CELLULAR_NETWORK_FAILED_REASON_UNKNOWN=1 # Error state is unknown
uint8 CELLULAR_NETWORK_FAILED_REASON_SIM_MISSING=2 # SIM is required for the modem but missing
uint8 CELLULAR_NETWORK_FAILED_REASON_SIM_ERROR=3 # SIM is available, but not usuable for connection

uint16 status 	# Status bitmap 1: Roaming is active
uint8 failure_reason #Failure reason when status in in CELLUAR_STATUS_FAILED
uint8 type 	# Cellular network radio type 0: none 1: gsm 2: cdma 3: wcdma 4: lte
uint8 quality	# Cellular network RSSI/RSRP in dBm, absolute value
uint16 mcc	# Mobile country code. If unknown, set to: UINT16_MAX
uint16 mnc	# Mobile network code. If unknown, set to: UINT16_MAX
uint16 lac	# Location area code. If unknown, set to: 0
