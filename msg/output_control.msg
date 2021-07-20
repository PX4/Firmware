uint64 timestamp         # time since system start (microseconds)
uint64 timestamp_sample  # the timestamp the data this control response is based on was sampled

uint8 NUM_OUTPUT_CONTROL_GROUPS = 5 # Max number of instances of this topic
uint8 MAX_ACTUATORS = 16            # Max actuators available in a single topic publication

float32[16] value         # Normalized control value in range [-1, 1]

# Actuator Function List
uint16 FUNCTION_NONE  = 0  # No output
uint16 FUNCTION_MIXER = 1  # Output comes from mixer file

# Misc. functions for different vehicle types
# You must implement a separate 'TOPICS' instance below for each of these you wish to use!
uint16 FUNCTION_PARACHUTE         = 10
uint16 FUNCTION_LANDING_GEAR      = 11
uint16 FUNCTION_LANDING_GEAR_DOOR = 12
uint16 FUNCTION_FLAPS             = 13
uint16 FUNCTION_SPOILER           = 14
uint16 FUNCTION_AIRBRAKS          = 15
uint16 FUNCTION_PAYLOAD_RELEASE   = 16

# Up to 8 individual actuators controlled by DO_SET_{SERVO,ACTUATOR}
uint16 FUNCTION_MAVLINK_SERVO0 = 100
uint16 FUNCTION_MAVLINK_SERVO1 = 101
uint16 FUNCTION_MAVLINK_SERVO2 = 102
uint16 FUNCTION_MAVLINK_SERVO3 = 103
uint16 FUNCTION_MAVLINK_SERVO4 = 104
uint16 FUNCTION_MAVLINK_SERVO5 = 105
uint16 FUNCTION_MAVLINK_SERVO6 = 106
uint16 FUNCTION_MAVLINK_SERVO7 = 107

# Specific multicopter functions
# This allows you to easily write your own mixer for multicopters
uint16 FUNCTION_MC_MOTOR1 = 200
uint16 FUNCTION_MC_MOTOR2 = 201
uint16 FUNCTION_MC_MOTOR3 = 202
uint16 FUNCTION_MC_MOTOR4 = 203
uint16 FUNCTION_MC_MOTOR5 = 204
uint16 FUNCTION_MC_MOTOR6 = 205
uint16 FUNCTION_MC_MOTOR7 = 206
uint16 FUNCTION_MC_MOTOR8 = 207

# Specific fixed-wing functions
# This allows you to easily write your own mixer for fixed-wings
uint16 FUNCTION_AILERON1  = 300
uint16 FUNCTION_AILERON2  = 301
uint16 FUNCTION_ELEVATOR1 = 302
uint16 FUNCTION_ELEVATOR2 = 303
uint16 FUNCTION_RUDDER1   = 304
uint16 FUNCTION_RUDDER2   = 305
uint16 FUNCTION_ENGINE1   = 306
uint16 FUNCTION_ENGINE2   = 307

# Gimbal / Camera Control
uint16 FUNCTION_GIMBAL_ROLL    = 400
uint16 FUNCTION_GIMBAL_PITCH   = 401
uint16 FUNCTION_GIMBAL_YAW     = 402
uint16 FUNCTION_GIMBAL_SHUTTER = 403
uint16 FUNCTION_CAMERA_ZOOM    = 404

# Functions mapping to outputs of the Control Allocation module
uint16 FUNCTION_CA0  = 1000
uint16 FUNCTION_CA1  = 1001
uint16 FUNCTION_CA2  = 1002
uint16 FUNCTION_CA3  = 1003
uint16 FUNCTION_CA4  = 1004
uint16 FUNCTION_CA5  = 1005
uint16 FUNCTION_CA6  = 1006
uint16 FUNCTION_CA7  = 1007
uint16 FUNCTION_CA8  = 1008
uint16 FUNCTION_CA9  = 1009
uint16 FUNCTION_CA10 = 1010
uint16 FUNCTION_CA11 = 1011
uint16 FUNCTION_CA12 = 1012
uint16 FUNCTION_CA13 = 1013
uint16 FUNCTION_CA14 = 1014
uint16 FUNCTION_CA15 = 1015

# TOPICS output_control output_control_ca output_control_mavlink output_control_mc output_control_fw
# TOPICS output_control_gimbal
