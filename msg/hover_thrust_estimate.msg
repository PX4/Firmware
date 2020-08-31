uint64 timestamp                # time since system start (microseconds)
uint64 timestamp_sample         # time of corresponding sensor data last used for this estimate

float32 hover_thrust		# estimated hover thrust [0.1, 0.9]
float32 hover_thrust_var	# estimated hover thrust variance

float32 accel_innov		# innovation of the last acceleration fusion
float32 accel_innov_var		# innovation variance of the last acceleration fusion
float32 accel_innov_test_ratio	# normalized innovation squared test ratio

float32 accel_noise_var		# vertical acceleration noise variance estimated form innovation residual

bool valid
