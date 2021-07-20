uint64 timestamp					# time since system start (microseconds)

uint8 STATE_STOPPED = 0					# The engine is not running. This is the default state.
uint8 STATE_STARTING = 1				# The engine is starting. This is a transient state.
uint8 STATE_RUNNING = 2					# The engine is running normally.
uint8 STATE_FAULT = 3					# The engine can no longer function.
uint8 state

uint32 FLAG_GENERAL_ERROR = 1				# General error.

uint32 FLAG_CRANKSHAFT_SENSOR_ERROR_SUPPORTED = 2	# Error of the crankshaft sensor. This flag is optional.
uint32 FLAG_CRANKSHAFT_SENSOR_ERROR = 4

uint32 FLAG_TEMPERATURE_SUPPORTED = 8			# Temperature levels. These flags are optional
uint32 FLAG_TEMPERATURE_BELOW_NOMINAL = 16      	# Under-temperature warning
uint32 FLAG_TEMPERATURE_ABOVE_NOMINAL = 32      	# Over-temperature warning
uint32 FLAG_TEMPERATURE_OVERHEATING = 64      		# Critical overheating
uint32 FLAG_TEMPERATURE_EGT_ABOVE_NOMINAL = 128     	# Exhaust gas over-temperature warning

uint32 FLAG_FUEL_PRESSURE_SUPPORTED = 256		# Fuel pressure. These flags are optional
uint32 FLAG_FUEL_PRESSURE_BELOW_NOMINAL  = 512     	# Under-pressure warning
uint32 FLAG_FUEL_PRESSURE_ABOVE_NOMINAL = 1024   	# Over-pressure warning

uint32 FLAG_DETONATION_SUPPORTED = 2048			# Detonation warning. This flag is optional.
uint32 FLAG_DETONATION_OBSERVED = 4096    		# Detonation condition observed warning

uint32 FLAG_MISFIRE_SUPPORTED = 8192			# Misfire warning. This flag is optional.
uint32 FLAG_MISFIRE_OBSERVED = 16384   			# Misfire condition observed warning

uint32 FLAG_OIL_PRESSURE_SUPPORTED = 32768		# Oil pressure. These flags are optional
uint32 FLAG_OIL_PRESSURE_BELOW_NOMINAL = 65536   	# Under-pressure warning
uint32 FLAG_OIL_PRESSURE_ABOVE_NOMINAL = 131072  	# Over-pressure warning

uint32 FLAG_DEBRIS_SUPPORTED = 262144			# Debris warning. This flag is optional
uint32 FLAG_DEBRIS_DETECTED = 524288  			# Detection of debris warning
uint32 flags

uint8 engine_load_percent				# Engine load estimate, percent, [0, 127]
uint32 engine_speed_rpm					# Engine speed, revolutions per minute
float32 spark_dwell_time_ms 				# Spark dwell time, millisecond
float32 atmospheric_pressure_kpa			# Atmospheric (barometric) pressure, kilopascal
float32 intake_manifold_pressure_kpa			# Engine intake manifold pressure, kilopascal
float32 intake_manifold_temperature			# Engine intake manifold temperature, kelvin
float32 coolant_temperature				# Engine coolant temperature, kelvin
float32 oil_pressure					# Oil pressure, kilopascal
float32 oil_temperature					# Oil temperature, kelvin
float32 fuel_pressure					# Fuel pressure, kilopascal
float32 fuel_consumption_rate_cm3pm			# Instant fuel consumption estimate, (centimeter^3)/minute
float32 estimated_consumed_fuel_volume_cm3		# Estimate of the consumed fuel since the start of the engine, centimeter^3
uint8 throttle_position_percent				# Throttle position, percent
uint8 ecu_index						# The index of the publishing ECU


uint8 SPARK_PLUG_SINGLE         = 0
uint8 SPARK_PLUG_FIRST_ACTIVE   = 1
uint8 SPARK_PLUG_SECOND_ACTIVE  = 2
uint8 SPARK_PLUG_BOTH_ACTIVE    = 3
uint8 spark_plug_usage					# Spark plug activity report.

float32 ignition_timing_deg				# Cylinder ignition timing, angular degrees of the crankshaft
float32 injection_time_ms				# Fuel injection time, millisecond
float32 cylinder_head_temperature			# Cylinder head temperature (CHT), kelvin
float32 exhaust_gas_temperature				# Exhaust gas temperature (EGT), kelvin
float32 lambda_coefficient				# Estimated lambda coefficient, dimensionless ratio
