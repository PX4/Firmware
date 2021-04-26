#!/usr/bin/env bash
set -eu

PATTERN="-e ."

if [ $# -gt 0 ]; then
    PATTERN="$1"
fi

exec find boards msg src platforms test \
    -path msg/templates/urtps -prune -o \
    -path platforms/nuttx/NuttX -prune -o \
    -path platforms/qurt/dspal -prune -o \
    -path src/drivers/uavcan/libuavcan -prune -o \
    -path src/drivers/uavcan_v1/libcanard -prune -o \
    -path src/drivers/uavcannode_gps_demo/libcanard -prune -o \
    -path src/drivers/uavcan/uavcan_drivers/kinetis/driver/include/uavcan_kinetis -prune -o \
    -path src/lib/ecl -prune -o \
    -path src/lib/matrix -prune -o \
    -path src/lib/parameters/uthash -prune -o \
    -path src/modules/gyro_fft/CMSIS_5 -prune -o \
    -path src/modules/micrortps_bridge/micro-CDR -prune -o \
    -path src/modules/micrortps_bridge/microRTPS_client -prune -o \
    -path test/mavsdk_tests/catch2 -prune -o \
    -type f \( -name "*.c" -o -name "*.h" -o -name "*.cpp" -o -name "*.hpp" \) | grep $PATTERN
