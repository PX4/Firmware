#!/bin/sh
#
# PX4IO interface init script.
#

# If $OUTPUT_MODE indicated Hardware-int-the-loop simulation, px4io should not publish actuator_outputs,
# instead, pwm_out_sim will publish that uORB
if [ $OUTPUT_MODE = hil ]
then
    set HIL_ARG $OUTPUT_MODE
fi

if [ $IO_PRESENT = yes ]
then
	if px4io start $HIL_ARG
	then
		# Allow PX4IO to recover from midair restarts.
		px4io recovery
	else
		echo "PX4IO start failed" >> $LOG_FILE
		tune_control play -t 20
	fi
fi
