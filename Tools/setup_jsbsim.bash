#!/bin/bash
#
# Setup environment to make JSBSim visible to PX4.
#
# License: according to LICENSE.md in the root directory of the PX4 Firmware repository

if [ "$#" != 3 ]; then
    echo -e "usage: source setup_jsbsim.bash src_dir build_dir model\n"
    return 1
fi

SRC_DIR="$1"
BUILD_DIR="$2"
MODEL="$3"

export FG_AIRCRAFT="${SRC_DIR}/Tools/jsbsim_bridge/models"

# This is needed for aircraft namespace mapping
# Need more architectural discussions to make this more scalable
case "$MODEL" in
        rascal)
            MODEL_NAME="Rascal110-JSBSim"
            ;;
        malolo)
            MODEL_NAME="Malolo1"
            ;;
        quadrotor_x)
            MODEL_NAME="quadrotor_x"
            ;;
        hexarotor_x)
            MODEL_NAME="hexarotor_x"
            ;;
        *)
            echo "Unknown Model"
            exit 1

esac

export JSBSIM_AIRCRAFT_MODEL="$MODEL_NAME"
