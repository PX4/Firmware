uint64 timestamp		# time since system start (microseconds)

# rate controller integrator status
float32 rollspeed_integ
float32 pitchspeed_integ
float32 yawspeed_integ
float32 additional_integ1	# FW: wheel rate integrator (optional)

# gain compression factors used for limit cycle protection
float32 roll_rate_gcf
float32 roll_angle_gcf
float32 pitch_rate_gcf
float32 pitch_angle_gcf
