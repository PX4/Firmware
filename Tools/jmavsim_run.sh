#!/usr/bin/env bash

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR/jMAVSim"

tcp_port=4560
extra_args=
baudrate=921600
device=
ip="127.0.0.1"
while getopts ":b:d:p:qsr:f:i:lo" opt; do
	case $opt in
		b)
			baudrate=$OPTARG
			;;
		d)
			device="$OPTARG"
			;;
		i)
			ip="$OPTARG"
			;;
		p)
			tcp_port=$OPTARG
			;;
		q)
			extra_args="$extra_args -qgc"
			;;
		s)
			extra_args="$extra_args -sdk"
			;;
		r)
			extra_args="$extra_args -r $OPTARG"
			;;
		l)
			extra_args="$extra_args -lockstep"
			;;
		o)
			extra_args="$extra_args -disponly"
			;;
		\?)
			echo "Invalid option: -$OPTARG" >&2
			exit 1
			;;
	esac
done

if [ "$device" == "" ]; then
	device="-tcp $ip:$tcp_port"
else
	device="-serial $device $baudrate"
fi

if [ "$HEADLESS" = "1" ]; then
    extra_args="$extra_args -no-gui"
fi

# jMAVSim crashes with Java 9 on macOS, therefore we need to use Java 8
if [ "$(uname)" == "Darwin" ]; then
    bold=$(tput bold)
    normal=$(tput sgr0)
    if ! /usr/libexec/java_home -V 2>&1 | grep --quiet "1.8\|Java SE 8" ; then
        echo "${bold}You need to have Java 8 installed for macOS, for more info, see:${normal}"
        echo "${bold}https://dev.px4.io/master/en/simulation/jmavsim.html#macos${normal}"
        exit 1
    fi
    export JAVA_HOME=`/usr/libexec/java_home -v 1.8`
elif [ "$(uname)" == "Linux" ]; then
    if ! java -version 2>&1 | grep --quiet "1.8" ; then
        echo "${bold}You need to use Java 8, for more info, see:${normal}"
        echo "${bold}https://dev.px4.io/master/en/simulation/jmavsim.html#ubuntu${normal}"
        exit 1
    fi
fi

ant create_run_jar copy_res
cd out/production

java -XX:GCTimeRatio=20 -Djava.ext.dirs= -Djavax.accessibility.assistive_technologies=" " -jar jmavsim_run.jar $device $extra_args
ret=$?
if [ $ret -ne 0 -a $ret -ne 130 ]; then # 130 is Ctrl-C
	# if the start of java fails, it's probably because the GC option is not
	# understood. Try starting without it
	java -Djava.ext.dirs= -Djavax.accessibility.assistive_technologies=" " -jar jmavsim_run.jar $device $extra_args
fi
