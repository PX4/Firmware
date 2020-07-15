uint64 timestamp			# time since system start (microseconds)
uint8 NUM_ACTUATOR_CONTROLS = 8
uint8 NUM_ACTUATOR_CONTROL_GROUPS = 6
uint8 INDEX_ROLL = 0
uint8 INDEX_PITCH = 1
uint8 INDEX_YAW = 2
uint8 INDEX_THROTTLE = 3
uint8 INDEX_FLAPS = 4
uint8 INDEX_SPOILERS = 5
uint8 INDEX_AIRBRAKES = 6
uint8 INDEX_LANDING_GEAR = 7
uint8 INDEX_GIMBAL_SHUTTER = 3
uint8 INDEX_CAMERA_ZOOM = 4

uint8 GROUP_INDEX_ATTITUDE = 0
uint8 GROUP_INDEX_ATTITUDE_ALTERNATE = 1
uint8 GROUP_INDEX_GIMBAL = 2
uint8 GROUP_INDEX_MANUAL_PASSTHROUGH = 3
uint8 GROUP_INDEX_ALLOCATED_PART1 = 4
uint8 GROUP_INDEX_ALLOCATED_PART2 = 5
uint8 GROUP_INDEX_PAYLOAD = 6

uint64 timestamp_sample	    # the timestamp the data this control response is based on was sampled
float32[8] control

# TOPICS actuator_controls actuator_controls_0 actuator_controls_1 actuator_controls_2 actuator_controls_3
# TOPICS actuator_controls_4 actuator_controls_5
# TOPICS actuator_controls_virtual_fw actuator_controls_virtual_mc
