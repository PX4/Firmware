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

# To disable user input
if [[ -n "$VERBOSE_SIM" ]]; then
	verbose="--verbose"
else
	verbose=""
fi

# Disable follow mode
if [[ "$PX4_NO_FOLLOW_MODE" != "1" ]]; then
    follow_mode="--gui-client-plugin libgazebo_user_camera_plugin.so"
else
    follow_mode=""
fi

if [ "$program" == "jmavsim" ]; then
	jmavsim_pid=`ps aux | grep java | grep "\-jar jmavsim_run.jar" | awk '{ print $2 }'`
	if [ -n "$jmavsim_pid" ]; then
		kill $jmavsim_pid
	fi
fi

if [ "$model" == "" ] || [ "$model" == "none" ]; then
	if [ "$program" == "jsbsim" ]; then
		echo "empty model, setting rascal as default for jsbsim"
		model="rascal"
	else
		echo "empty model, setting iris as default"
		model="iris"
	fi
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

export PX4_SIM_MODEL=${model}

SIM_PID=0

if [ "$program" == "jmavsim" ] && [ ! -n "$no_sim" ]; then
	# Start Java simulator
	"$src_path"/Tools/jmavsim_run.sh -r 250 -l &
	SIM_PID=$!
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
					world_path="${src_path}/Tools/sitl_gazebo/worlds/${model}.world"
				else
					echo "empty world, setting empty.world as default"
					world_path="${src_path}/Tools/sitl_gazebo/worlds/empty.world"
				fi
			else
				#Spawn empty world if world with model name doesn't exist
				world_path="${src_path}/Tools/sitl_gazebo/worlds/${world}.world"
			fi
		else
			if [ -f ${src_path}/Tools/sitl_gazebo/worlds/${PX4_SITL_WORLD}.world ]; then
				# Spawn world by name if exists in the worlds directory from environment variable
				world_path="${src_path}/Tools/sitl_gazebo/worlds/${PX4_SITL_WORLD}.world"
			else
				# Spawn world from environment variable with absolute path
				world_path="$PX4_SITL_WORLD"
			fi
		fi
		gzserver $verbose $world_path &
		SIM_PID=$!

		# Check all paths in ${GAZEBO_MODEL_PATH} for specified model
		IFS_bak=$IFS
		IFS=":"
		for possible_model_path in ${GAZEBO_MODEL_PATH}; do
			if [ -z $possible_model_path ]; then
				continue
			fi
			# trim \r from path
			possible_model_path=$(echo $possible_model_path | tr -d '\r')
			if test -f "${possible_model_path}/${model}/${model}.sdf" ; then
				modelpath=$possible_model_path
				break
			fi
		done
		IFS=$IFS_bak

		if [ -z $modelpath ]; then
			echo "Model ${model} not found in model path: ${GAZEBO_MODEL_PATH}"
			exit 1
		else
			echo "Using: ${modelpath}/${model}/${model}.sdf"
		fi

		while gz model --verbose --spawn-file="${modelpath}/${model}/${model_name}.sdf" --model-name=${model} -x 1.01 -y 0.98 -z 0.83 2>&1 | grep -q "An instance of Gazebo is not running."; do
			echo "gzserver not ready yet, trying again!"
			sleep 1
		done

		if [[ -n "$HEADLESS" ]]; then
			echo "not running gazebo gui"
		else
			# gzserver needs to be running to avoid a race. Since the launch
			# is putting it into the background we need to avoid it by backing off
			sleep 3
			nice -n 20 gzclient --verbose $follow_mode &
			GUI_PID=$!
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
	FG_BRIDGE_PID=$!
elif [ "$program" == "jsbsim" ] && [ -z "$no_sim" ]; then
	source "$src_path/Tools/setup_jsbsim.bash" "${src_path}" "${build_path}" ${model}
	if [[ -n "$HEADLESS" ]]; then
		echo "not running flightgear gui"
	else
		fgfs --fdm=null \
			--native-fdm=socket,in,60,,5550,udp \
			--aircraft=$JSBSIM_AIRCRAFT_MODEL \
			--airport=${world} \
			--disable-hud \
			--disable-ai-models &> /dev/null &
		FGFS_PID=$!
	fi
	"${build_path}/build_jsbsim_bridge/jsbsim_bridge" ${model} -s "${src_path}/Tools/jsbsim_bridge/scene/${world}.xml" 2> /dev/null &
	JSBSIM_PID=$!
fi

pushd "$rootfs" >/dev/null

# Do not exit on failure now from here on because we want the complete cleanup
set +e

if [[ ${model} == test_* ]] || [[ ${model} == *_generated ]]; then
	sitl_command="\"$sitl_bin\" $no_pxh \"$src_path\"/ROMFS/px4fmu_common -s \"${src_path}\"/posix-configs/SITL/init/test/${model} -t \"$src_path\"/test_data"
else
	sitl_command="\"$sitl_bin\" $no_pxh \"$build_path\"/etc -s etc/init.d-posix/rcS -t \"$src_path\"/test_data"
fi

echo SITL COMMAND: $sitl_command

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
elif [ "$program" == "jsbsim" ]; then
	kill $JSBSIM_PID
	kill $FGFS_PID
fi
