uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample         # the timestamp of the raw data (microseconds)

# GPS
float32[2] gps_hvel	# horizontal GPS velocity innovation (m/sec) and innovation variance ((m/sec)**2)
float32    gps_vvel	# vertical GPS velocity innovation (m/sec) and innovation variance ((m/sec)**2)
float32[2] gps_hpos	# horizontal GPS position innovation (m) and innovation variance (m**2)
float32    gps_vpos	# vertical GPS position innovation (m) and innovation variance (m**2)

# External Vision
float32[2] ev_hvel	# horizontal external vision velocity innovation (m/sec) and innovation variance ((m/sec)**2)
float32    ev_vvel	# vertical external vision velocity innovation (m/sec) and innovation variance ((m/sec)**2)
float32[2] ev_hpos	# horizontal external vision position innovation (m) and innovation variance (m**2)
float32    ev_vpos	# vertical external vision position innovation (m) and innovation variance (m**2)

# Height sensors
float32 rng_vpos	# range sensor height innovation (m) and innovation variance (m**2)
float32 baro_vpos	# barometer height innovation (m) and innovation variance (m**2)

# Auxiliary velocity
float32[2] aux_hvel	# horizontal auxiliar velocity innovation from landing target measurement (m/sec) and innovation variance ((m/sec)**2)
float32    aux_vvel	# vertical auxiliar velocity innovation from landing target measurement (m/sec) and innovation variance ((m/sec)**2)

# Optical flow
float32[2] flow		# flow innvoation (rad/sec) and innovation variance ((rad/sec)**2)

# Various
float32 heading		# heading innovation (rad) and innovation variance (rad**2)
float32[3] mag_field	# earth magnetic field innovation (Gauss) and innovation variance (Gauss**2)
float32[2] drag		# drag specific force innovation (m/sec**2) and innovation variance ((m/sec)**2)
float32 airspeed	# airspeed innovation (m/sec) and innovation variance ((m/sec)**2)
float32 beta		# synthetic sideslip innovation (rad) and innovation variance (rad**2)
float32 hagl		# height of ground innovation (m) and innovation variance (m**2)

# The innovation test ratios are scalar values. In case the field is a vector,
# the test ratio will be put in the first component of the vector.

# TOPICS estimator_innovations estimator_innovation_variances estimator_innovation_test_ratios
