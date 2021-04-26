
# Excelsior is the code name of a board currently in development.

include(px4_git)
px4_add_git_submodule(TARGET git_cmake_hexagon PATH "${PX4_SOURCE_DIR}/boards/atlflight/cmake_hexagon")
list(APPEND CMAKE_MODULE_PATH
	"${PX4_SOURCE_DIR}/boards/atlflight/cmake_hexagon"
	"${PX4_SOURCE_DIR}/boards/atlflight/cmake_hexagon/toolchain"
	)

# Get $QC_SOC_TARGET from environment if existing.
if (DEFINED ENV{QC_SOC_TARGET})
	set(QC_SOC_TARGET $ENV{QC_SOC_TARGET})
else()
	set(QC_SOC_TARGET "APQ8074")
endif()

# Disable the creation of the parameters.xml file by scanning individual
# source files, and scan all source files.  This will create a parameters.xml
# file that contains all possible parameters, even if the associated module
# is not used.  This is necessary for parameter synchronization between the
# ARM and DSP processors.
set(DISABLE_PARAMS_MODULE_SCOPING TRUE)

set(CONFIG_SHMEM "1")
add_definitions(-DORB_COMMUNICATOR)

# atlflight toolchain doesn't properly set the compiler, so these aren't set automatically
add_compile_options($<$<COMPILE_LANGUAGE:C>:-std=gnu99>)
add_compile_options($<$<COMPILE_LANGUAGE:CXX>:-std=gnu++11>)

add_definitions(
	-D__PX4_POSIX_EXCELSIOR
	-D__PX4_LINUX
)

px4_add_board(
	PLATFORM posix
	VENDOR atlflight
	MODEL excelsior
	LABEL default
	TOOLCHAIN arm-oemllib32-linux-gnueabi
	DRIVERS
		#barometer # all available barometer drivers
		batt_smbus
		camera_trigger
		differential_pressure # all available differential pressure drivers
		distance_sensor # all available distance sensor drivers
		gps
		#imu # all available imu drivers
		#lights/rgbled
		#magnetometer # all available magnetometer drivers
		pwm_out_sim
		qshell/posix
		rc_input
		#telemetry # all available telemetry drivers
	MODULES
		airspeed_selector
		attitude_estimator_q
		camera_feedback
		commander
		dataman
		ekf2
		events
		flight_mode_manager
		fw_att_control
		fw_pos_control_l1
		gyro_calibration
		gyro_fft
		land_detector
		landing_target_estimator
		#load_mon
		local_position_estimator
		logger
		mavlink
		mc_att_control
		mc_hover_thrust_estimator
		mc_pos_control
		mc_rate_control
		#micrortps_bridge
		muorb/krait
		muorb/test
		navigator
		rc_update
		rover_pos_control
		sensors
		#sih
		simulator
		vmount
		vtol_att_control
	SYSTEMCMDS
		#bl_update
		#dumpfile
		esc_calib
		#hardfault_log
		led_control
		mixer
		motor_ramp
		motor_test
		#mtd
		#nshterm
		param
		perf
		pwm
		sd_bench
		shutdown
		system_time
		#top
		topic_listener
		tune_control
		uorb
		ver
		work_queue
	EXAMPLES
		#fake_gps
		#fixedwing_control # Tutorial code from https://px4.io/dev/example_fixedwing_control
		#hello
		#hwtest # Hardware test
		#matlab_csv_serial
		#px4_mavlink_debug # Tutorial code from http://dev.px4.io/en/debug/debug_values.html
		#px4_simple_app # Tutorial code from http://dev.px4.io/en/apps/hello_sky.html
		#rover_steering_control # Rover example app
		#uuv_example_app
		#work_item
	)
