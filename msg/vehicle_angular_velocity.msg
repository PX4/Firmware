
uint64 timestamp        # time since system start (microseconds)
uint64 timestamp_sample # timestamp of the data sample on which this message is based (microseconds)

float32[3] xyz		# Bias corrected angular velocity about the FRD body frame XYZ-axis in rad/s

# TOPICS vehicle_angular_velocity vehicle_angular_velocity_groundtruth
