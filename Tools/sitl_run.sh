#!/bin/bash

set -e

if [ "$#" -lt 6 ]; then
	echo usage: sitl_run.sh sitl_bin debugger program model src_path build_path
	exit 1
fi

if [[ -n "$DONT_RUN" ]]; then
	echo "Not running simulation (DONT_RUN is set)."
    exit 0
fi

sitl_bin="$1"
debugger="$2"
program="$3"
model="$4"
src_path="$5"
build_path="$6"
# The rest of the arguments are files to copy into the working dir.

echo SITL ARGS

echo sitl_bin: $sitl_bin
echo debugger: $debugger
echo program: $program
echo model: $model
echo src_path: $src_path
echo build_path: $build_path

rootfs="$build_path/tmp/rootfs" # this is the working directory
mkdir -p "$rootfs"

# To disable user input
if [[ -n "$NO_PXH" ]]; then
	no_pxh=-d
else
	no_pxh=""
fi

if [ "$model" != none ]; then
	jmavsim_pid=`ps aux | grep java | grep "\-jar jmavsim_run.jar" | awk '{ print $2 }'`
	if [ -n "$jmavsim_pid" ]; then
		kill $jmavsim_pid
	fi
fi

if [ "$model" == "" ] || [ "$model" == "none" ]; then
	echo "empty model, setting iris as default"
	model="iris"
fi

# kill process names that might stil
# be running from last time
pkill -x gazebo || true

# Do NOT kill PX4 if debug in ide
if [ "$debugger" != "ide" ]; then
	pkill -x px4 || true
	pkill -x px4_$model || true
fi

cp "$src_path/Tools/posix_lldbinit" "$rootfs/.lldbinit"
cp "$src_path/Tools/posix.gdbinit" "$rootfs/.gdbinit"

shift 6
for file in "$@"; do
	cp "$file" $rootfs/
done

SIM_PID=0

if [ "$program" == "jmavsim" ] && [ ! -n "$no_sim" ]; then
	# Start Java simulator
	"$src_path"/Tools/jmavsim_run.sh -r 250 -l &
	SIM_PID=`echo $!`
elif [ "$program" == "gazebo" ] && [ ! -n "$no_sim" ]; then
	if [ -x "$(command -v gazebo)" ]; then
	# Set the plugin path so Gazebo finds our model and sim
	source "$src_path/Tools/setup_gazebo.bash" "${src_path}" "${build_path}"

			gzserver "${src_path}/Tools/sitl_gazebo/worlds/${model}.world" &
			SIM_PID=`echo $!`

	if [[ -n "$HEADLESS" ]]; then
		echo "not running gazebo gui"
	else
		# gzserver needs to be running to avoid a race. Since the launch
		# is putting it into the background we need to avoid it by backing off
		sleep 3
		nice -n 20 gzclient --verbose &
		GUI_PID=`echo $!`
		fi
	else
		echo "You need to have gazebo simulator installed!"
		exit 1
	fi
fi

pushd "$rootfs" >/dev/null

# Do not exit on failure now from here on because we want the complete cleanup
set +e

if [[ ${model} == test_* ]] || [[ ${model} == *_generated ]]; then
	sitl_command="\"$sitl_bin\" $no_pxh \"$src_path\"/ROMFS/px4fmu_test -s \"${src_path}\"/posix-configs/SITL/init/test/${model} -t \"$src_path\"/test_data"
else
	sitl_command="\"$sitl_bin\" $no_pxh \"$src_path\"/ROMFS/px4fmu_common -s etc/init.d-posix/rcS -t \"$src_path\"/test_data"
fi

echo SITL COMMAND: $sitl_command

export PX4_SIM_MODEL=${model}


if [ "$debugger" == "lldb" ]; then
	eval lldb -- $sitl_command
elif [ "$debugger" == "gdb" ]; then
	eval gdb --args $sitl_command
elif [ "$debugger" == "ddd" ]; then
	eval ddd --debugger gdb --args $sitl_command
elif [ "$debugger" == "valgrind" ]; then
	eval valgrind --track-origins=yes --leak-check=full -v $sitl_command
elif [ "$debugger" == "callgrind" ]; then
	eval valgrind --tool=callgrind -v $sitl_command
elif [ "$debugger" == "ide" ]; then
	echo "######################################################################"
	echo
	echo "PX4 simulator not started, use your IDE to start PX4_${model} target."
	echo "Hit enter to quit..."
	echo
	echo "######################################################################"
	read
else
	eval $sitl_command
fi

popd >/dev/null

if [ "$program" == "jmavsim" ]; then
	pkill -9 -P $SIM_PID
	kill -9 $SIM_PID
elif [ "$program" == "gazebo" ]; then
	kill -9 $SIM_PID
	if [[ ! -n "$HEADLESS" ]]; then
		kill -9 $GUI_PID
	fi
fi
