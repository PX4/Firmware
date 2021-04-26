uint64 timestamp		# time since system start (microseconds)
uint64 timestamp_sample         # the timestamp of the raw data (microseconds)

float32 windspeed_north		# Wind component in north / X direction (m/sec)
float32 windspeed_east		# Wind component in east / Y direction (m/sec)

float32 variance_north		# Wind estimate error variance in north / X direction (m/sec)**2 - set to zero (no uncertainty) if not estimated
float32 variance_east		# Wind estimate error variance in east / Y direction (m/sec)**2 - set to zero (no uncertainty) if not estimated

float32 tas_innov 		# True airspeed innovation
float32 tas_innov_var 		# True airspeed innovation variance

float32 beta_innov 		# Sideslip measurement innovation
float32 beta_innov_var 		# Sideslip measurement innovation variance

# TOPICS wind estimator_wind
