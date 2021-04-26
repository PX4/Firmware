#!/bin/bash
#Author: Benjamin Perseghetti
#Email: bperseghetti@rudislabs.com
# This script unifies running gazebo simulation for HITL and SITL
# You can run multiple instances of the 'px4' binary, with the gazebo SITL simulation
# This script assumes px4 is already built, with 'make px4_sitl_default gazebo'
# You can also run HITL with -h flag 
# Generate world and/or model files with editable json -j [m (model), w (world), mw (model and world), or wm (world and model)]

# The simulator in SITL is expected to send to TCP port 4560+i for i in [0, N-1]
# For example gazebo can be run like this for multiple SITL:
# ./Tools/gz_sim.sh -n 10 -m iris
# Or gazebo can be run like this for HITL:
# ./Tools/gz_sim.sh -h 1 -m standard_vtol

function cleanup() {
	echo "running the cleanup"
	pkill -x px4
	pkill gazebo
	pkill gzclient
	pkill gzserver
}

trap "cleanup" INT SIGINT SIGTERM EXIT

function spawn_model() {
	MODEL=$1
	N=$2 #Instance Number
	WORLD_FILE=$3
	MJ=$4
	SITL_MODEL_NAME="${MODEL}_${N}"
	sitl_path=${SCRIPT_DIR}/sitl_gazebo
	jinja_model_script=${sitl_path}/scripts/jinja_model_gen.py
	base_model="--base_model ${MODEL}"
	model_json="--json_gen ${MJ}"

	if [ $hitl == true ]; then
		python3 ${src_path}/Tools/boot_now.py "/dev/ttyACM0"
		hil_mode="--hil_mode 1"
		model_name="--model_name ${MODEL}"
		hitl_launch_command="${model_json} ${base_model} ${hil_mode}  ${model_name}"
		echo "Generating: ${jinja_model_script} ${hitl_launch_command}"
		python3 ${jinja_model_script} ${hitl_launch_command}
		sleep 1
		source ${src_path}/Tools/setup_gazebo.bash ${src_path} ${src_path}/build/${target}
		sleep 2
		gazebo ${sitl_path}/worlds/temp_${WORLD_FILE}.world --verbose

	else
		mavlink_tcp="--mavlink_tcp_port $((4560+${N}))"
		mavlink_udp="--mavlink_udp_port $((14560+${N}))"
		model_name="--model_name ${SITL_MODEL_NAME}"
		output_path="--output_path /tmp"
		working_dir="$build_path/instance_$n"
		sitl_launch_command="${model_json} ${base_model} ${mavlink_tcp} ${mavlink_udp} ${model_name} ${output_path}"
		[ ! -d "$working_dir" ] && mkdir -p "$working_dir"
		pushd "$working_dir" &>/dev/null
		echo "starting instance $N in $(pwd)"
		../bin/px4 -i $N -d "$build_path/etc" -w sitl_${SITL_MODEL_NAME} -s etc/init.d-posix/rcS >out.log 2>err.log &
		python3 ${jinja_model_script} ${sitl_launch_command}
		echo "Generating: ${jinja_model_script} ${sitl_launch_command}" 
		echo "Spawning ${SITL_MODEL_NAME}"
		gz model --spawn-file=/tmp/${SITL_MODEL_NAME}.sdf --model-name=${SITL_MODEL_NAME} -x 0.0 -y $((3*${N})) -z 0.2	
		popd &>/dev/null
	fi
	
}

if [ "$1" == "--help" ]; then
	echo "Usage: $0 [-n <num_vehicles>] [-m <vehicle_model>] [-h <run_hitl>] [-w <world>] [-s <script>] [-t <num_threads>] [-j <json_params>]"
	echo "-s flag is used to script spawning vehicles e.g. $0 -s iris:3,plane:2"
	echo "-h flag is used to launch a single vehicle in HITL mode"
	echo "-t flag is used to set the number of ODE threads for the world"
	echo "-j flag is used to enable json parameters from gen_params.json for the world (w), model (m), or both (wm or mw)"
	exit 1
fi

while getopts n:m:h:w:s:t:j:p option
do
	case "${option}"
	in
		n) NUM_VEHICLES=${OPTARG};;
		m) VEHICLE_MODEL=${OPTARG};;
		h) HITL=${OPTARG};;
		w) WORLD=${OPTARG};;
		s) SCRIPT=${OPTARG};;
		t) THREADS=${OPTARG};;
		j) JSON=${OPTARG};;
		p) TARGET=${OPTARG};;
	esac
done

num_vehicles=${NUM_VEHICLES:=1}
world=${WORLD:=empty}
hitl=${HITL:=false}
threads=${THREADS:=1}
json_opts=${JSON:=0}
target=${TARGET:=px4_sitl_default}
system_threads=`grep -Pc '^processor\t' /proc/cpuinfo`
echo "Number of requested ODE threads: $((threads))"
echo "Max number of possilbe threads: $((system_threads))"
if [ $((threads)) -gt $((system_threads)) ]; then
	threads=$system_threads
	echo "Requested ODE thread count too high, set to system max of $threads threads."
elif [ $(( ${threads} )) -lt 1 ]; then
	threads=1
	echo "Requested ODE thread count too low, set to $threads thread."
else
	echo "Using $threads threads for ODE."
fi

if [ "$json_opts" == "mw" ] || [ "$json_opts" == "wm" ]; then
	echo "JSON used for both world and model generation"
	wjson="1"
	mjson="1"
elif [ "$json_opts" == "w" ]; then
	echo "JSON used for world generation"
	wjson="1"
	mjson="0"
elif [ "$json_opts" == "m" ]; then
	echo "JSON used for model generation"
	mjson="1"
	wjson="0"
else 
	wjson="0"
	mjson="0"
fi

if [ "$hitl" == "True" ] || [ "$hitl" == "1" ] || [ "$hitl" == "true" ]; then
	hitl=true
else
	hitl=false
fi

export PX4_SIM_MODEL=${VEHICLE_MODEL:=iris}

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
src_path="$SCRIPT_DIR/.."
echo ${SCRIPT}
build_path=${src_path}/build/${target}
source ${src_path}/Tools/setup_gazebo.bash ${src_path} ${src_path}/build/${target}
sleep 1

sitl_path=${SCRIPT_DIR}/sitl_gazebo
world_name="--world_name ${world}"
jinja_world_script=${sitl_path}/scripts/jinja_world_gen.py
sitl_ode_threads="--ode_threads ${threads}"
world_json="--json_gen ${wjson}"

if [ $hitl == true ]; then
	hitl_model_name="--model_name ${PX4_SIM_MODEL}"
	echo "HITL mode is currently turned on, disabling multiple vehicle spawn and script spawn."
	echo "RUNNING: python3 $jinja_world_script $world_name $hitl_model_name $world_json"
	python3 $jinja_world_script $world_name $hitl_model_name $world_json
	echo "Generated temp_${world}.world"
	spawn_model ${PX4_SIM_MODEL} 0 ${world} ${mjson}

else
	echo "killing running instances"
	pkill -x px4 || true
	echo "HITL mode is currently turned off."
	echo "RUNNING: python3 $jinja_world_script $world_name $sitl_ode_threads $world_json"
	python3 $jinja_world_script $world_name $sitl_ode_threads $world_json
	echo "Generated temp_${world}.world"
	echo "Starting gazebo: gzserver ${sitl_path}/worlds/temp_${world}.world --verbose"
	gzserver ${sitl_path}/worlds/temp_${world}.world --verbose &
	sleep 5

	n=0
	if [ -z ${SCRIPT} ]; then
		if [ $num_vehicles -gt 255 ]; then
			echo "Tried spawning $num_vehicles vehicles. The maximum number of supported vehicles is 255"
			exit 1
		fi
		while [ $n -lt $num_vehicles ]; do
			spawn_model ${PX4_SIM_MODEL} $n ${world} ${mjson}
			n=$(($n + 1))
		done
	else
		for target in ${SCRIPT}; do
			target="$(echo "$target" | tr -d ' ')" #Remove spaces
			target_vehicle="${target%:*}"
			target_number="${target#*:}"

			if [ $n -gt 255 ]; then
				echo "Tried spawning $n vehicles. The maximum number of supported vehicles is 255"
				exit 1
			fi

			m=0
			while [ $m -lt ${target_number} ]; do
				spawn_model ${PX4_SIM_MODEL} $n ${world} ${mjson}
				m=$(($m + 1))
				n=$(($n + 1))
			done
		done
	fi
	echo "Starting gazebo client"
	gzclient
fi
