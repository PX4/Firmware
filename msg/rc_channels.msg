uint64 timestamp						# time since system start (microseconds)

uint8 FUNCTION_THROTTLE   = 0
uint8 FUNCTION_ROLL       = 1
uint8 FUNCTION_PITCH      = 2
uint8 FUNCTION_YAW        = 3
uint8 FUNCTION_RES1       = 4
uint8 FUNCTION_RETURN     = 5
uint8 FUNCTION_RES2       = 6
uint8 FUNCTION_LOITER     = 7
uint8 FUNCTION_OFFBOARD   = 8
uint8 FUNCTION_RES3       = 9
uint8 FUNCTION_FLAPS      = 10
uint8 FUNCTION_AUX_1      = 11
uint8 FUNCTION_AUX_2      = 12
uint8 FUNCTION_AUX_3      = 13
uint8 FUNCTION_AUX_4      = 14
uint8 FUNCTION_AUX_5      = 15
uint8 FUNCTION_PARAM_1    = 16
uint8 FUNCTION_PARAM_2    = 17
uint8 FUNCTION_PARAM_3_5  = 18
uint8 FUNCTION_KILLSWITCH = 19
uint8 FUNCTION_TRANSITION = 20
uint8 FUNCTION_GEAR       = 21
uint8 FUNCTION_ARMSWITCH  = 22
uint8 FUNCTION_RES4       = 23
uint8 FUNCTION_AUX_6      = 24
uint8 FUNCTION_RES5       = 25

uint64 timestamp_last_valid					# Timestamp of last valid RC signal
float32[18] channels						# Scaled to -1..1 (throttle: 0..1)
uint8 channel_count						# Number of valid channels
int8[26] function						# Functions mapping
uint8 rssi							# Receive signal strength index
bool signal_lost						# Control signal lost, should be checked together with topic timeout
uint32 frame_drop_count						# Number of dropped frames
