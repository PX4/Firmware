#!/usr/bin/env bash


set -e

if [ "$#" -lt 7 ]; then
	echo usage: sitl_run.sh sitl_bin debugger program model world src_path build_path
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
world="$5"
src_path="$6"
build_path="$7"
# The rest of the arguments are files to copy into the working dir.

echo SITL ARGS

echo sitl_bin: $sitl_bin
echo debugger: $debugger
echo program: $program
echo model: $model
echo world: $world
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

shift 7
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
		# Get the model name
		model_name="${model}"
		# Check if a 'modelname-gen.sdf' file exist for the models using jinja and generating the SDF files
		if [ -f "${src_path}/Tools/sitl_gazebo/models/${model}/${model}-gen.sdf" ]; then
			model_name="${model}-gen"
		fi

		# Set the plugin path so Gazebo finds our model and sim
		source "$src_path/Tools/setup_gazebo.bash" "${src_path}" "${build_path}"
		if [ -z $PX4_SITL_WORLD ]; then
			#Spawn predefined world
			if [ "$world" == "none" ]; then
				if [ -f ${src_path}/Tools/sitl_gazebo/worlds/${model}.world ]; then
					echo "empty world, default world ${model}.world for model found"
					gzserver "${src_path}/Tools/sitl_gazebo/worlds/${model}.world" &
				else
					echo "empty world, setting empty.world as default"
					gzserver "${src_path}/Tools/sitl_gazebo/worlds/empty.world" &
				fi
			else
				#Spawn empty world if world with model name doesn't exist
				gzserver "${src_path}/Tools/sitl_gazebo/worlds/${world}.world" &
			fi
		else
			if [ -f ${src_path}/Tools/sitl_gazebo/worlds/${PX4_SITL_WORLD}.world ]; then
				# Spawn world by name if exists in the worlds directory from environment variable
				gzserver "${src_path}/Tools/sitl_gazebo/worlds/${PX4_SITL_WORLD}.world" &
			else
				# Spawn world from environment variable with absolute path
				gzserver "$PX4_SITL_WORLD" &
			fi
		fi
		SIM_PID=$!

		while gz model --verbose --spawn-file="${src_path}/Tools/sitl_gazebo/models/${model}/${model_name}.sdf" --model-name=${model} -x 1.01 -y 0.98 -z 0.83 2>&1 | grep -q "An instance of Gazebo is not running."; do
			echo "gzserver not ready yet, trying again!"
			sleep 1
		done

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
elif [ "$program" == "flightgear" ] && [ -z "$no_sim" ]; then
	echo "FG setup"
	cd "${src_path}/Tools/flightgear_bridge/"
	"${src_path}/Tools/flightgear_bridge/FG_run.py" "models/"${model}".json" 0
	"${build_path}/build_flightgear_bridge/flightgear_bridge" 0 `./get_FGbridge_params.py "models/"${model}".json"` &
	FG_BRIDGE_PID=`echo $!`
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
elif [ "$program" == "flightgear" ]; then
	kill $FG_BRIDGE_PID
	kill -9 `cat /tmp/px4fgfspid_0`
fi
