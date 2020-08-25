#!/bin/bash

#################################################################################################
#
# This script loads PX4 binaries to the Snapdragon Flight target and does a quick on-target sanity test.
#
# Pre-requisites:
#  - Snapdragon Flight board connected to the host computer via USB cable
#  - Snapdragon Flight board must have the latest platform BSP and flight controller addon installed"
#  - mini-dm installed on host computer (see https://github.com/ATLFlight/ATLFlightDocs/blob/master/UserGuide.md#adsp)
#  - PX4 software was built and binaries are in their usual locations in the Firmware tree.
#
# This script supports two modes:
#  - Default mode (supported by PX4 community)
#  - Legacy mode (uses proprietary drivers for ESC and RC Receiver, supported by Qualcomm)
#
# For help and cmd line options, run the script with the -h option
#
#################################################################################################

# Halt on error
set -e

# Verbose mode
## set -x

# Mode of operation
readonly MODE_DEFAULT=0
readonly MODE_LEGACY=1
readonly MODE_8x96=2
readonly MODE_MAX=$MODE_8x96

readonly RESULT_PASS=0
readonly RESULT_FAIL=3
readonly EXIT_ERROR=3


# List of expected strings from the apps proc
declare -a appsproc_strings_present=(
   "on udp port 14556 remote port 14550"
   )

# List of unexpected strings from the apps proc
declare -a appsproc_strings_absent=(
   "ERROR"
   "Getting Bulk data from fastRPC link"
   "Segmentation fault"
   )

# List of expected strings from the DSP
declare -a dsp_strings_present=(
    "EKF aligned"
   )

# List of unexpected strings from the DSP
declare -a dsp_strings_absent=(
   "Segmentation fault"
   )


install=0
test=0
mode=0
result=$RESULT_PASS

# Default mini-dm path (needs to be installed in this location or overriden through cmd line
minidmPath=~/Qualcomm/Hexagon_SDK/3.0/tools/debug/mini-dm/Linux_Debug

# Default workspace path (parent directory of the script location)
workspace=`pwd`/../../../..


verifypx4test() {

   #TODO: This needs to be fixed. For now, skip string checks for 8x96 platform.
   if [ $mode == 2 ]; then
      echo -e "[WARNING] Skipping string checks for 8x96 platform"
      return
   fi

   echo -e "Verifying test results..."

   # verify the presence of expected stings in the apps proc console log
   for lineString in "${appsproc_strings_present[@]}"
   do
   	if ! grep -Fq "$lineString" px4.log
   	then
         # code if not found
         echo -e "[ERROR] Missing expected string in apps proc log: $lineString"
         result=$RESULT_FAIL
      fi
   done

   # verify the absence of unexpected stings in the apps proc console log
   for lineString in "${appsproc_strings_absent[@]}"
   do
   	if grep -Fq "$lineString" px4.log
   	then
         # code if not found
         echo -e "[ERROR] Found unexpected string in apps proc log: $lineString"
         result=$RESULT_FAIL
      fi
   done

   echo -e "Displaying the content of the minidm.log"
   cat minidm.log
   echo -e "Analyzing the log for success and failure indications."

   # verify the presence of expected stings in the DSP console log
   for lineString in "${dsp_strings_present[@]}"
   do
   	if ! grep -Fq "$lineString" minidm.log
   	then
         # code if not found
         echo -e "[ERROR] Missing expected string in DSP log: $lineString"
         result=$RESULT_FAIL
      fi
   done

   # verify the absence of unexpected stings in the DSP console log
   for lineString in "${dsp_strings_absent[@]}"
   do
   	if grep -Fq "$lineString" minidm.log
   	then
         # code if not found
         echo -e "[ERROR] Found unexpected string in DSP log: $lineString"
         result=$RESULT_FAIL
      fi
   done

   echo -e "Verification complete."

   if [ $result -eq $RESULT_FAIL ]; then
      echo -e "PX4 test result: FAIL"
   else
      echo -e "PX4 test result: PASS"
   fi
}

installpx4() {

   if [ $install -eq 0 ]; then
      echo -e "SKIPPING install"
      return 0;
   fi

   # Reboot the target before beginning the installation
   echo -e "Rebooting the target..."
   adb reboot
   adb wait-for-usb-device
   # Wait a bit longer after bootup, before copying binaries to the target.
   sleep 30
   adb devices

   echo -e "Now installing PX4 binaries..."
   # Copy binaries to the target
   if [ $mode == 0 ]; then
      # copy default binaries
      echo -e "Copying the PX4 binaries from the eagle_default build tree..."
      adb push $workspace/build/atlflight_eagle_qurt/platforms/qurt/libpx4.so /usr/share/data/adsp
      adb push $workspace/build/atlflight_eagle_qurt/platforms/qurt/libpx4muorb_skel.so /usr/share/data/adsp
      adb push $workspace/build/atlflight_eagle_default/bin/px4 /home/linaro
      adb push $workspace/posix-configs/eagle/flight/px4.config /usr/share/data/adsp
      adb push $workspace/posix-configs/eagle/flight/mainapp.config /home/linaro
   fi

   echo -e "Installation complete."
}


testpx4() {

   if [ $test -eq 0 ]; then
      echo -e "SKIPPING test"
      return 0;
   fi

   echo -e "Starting PX4 test..."

   # Remove previous instances of the file
   rm px4.log | true
   rm minidm.log | true

   # Start mini-dm
   echo -e "Starting mini-dm..."
   ${minidmPath}/mini-dm > minidm.log &
   sleep 5
   # Verify that mini-dm is running
   checkProc=$(ps -aef | grep mini-dm | grep -v grep)

   if [ -z "${checkProc}" ]; then
      echo "[ERROR] Unable to start mini-dm from path: ${minidmPath}"
      exit $EXIT_ERROR
   fi


   # Start PX4
   echo -e "Starting PX4..."
   if [ $mode == 2 ]; then
      # 8x96 platform
      adb shell "/home/root/px4 /home/root/mainapp.config" > px4.log 2>&1 &
   else
      # 8x74 platform
      adb shell "/home/linaro/px4 /home/linaro/mainapp.config" > px4.log 2>&1 &
   fi
   sleep 20
   # Verify that PX4 is still running
   checkProc=$(adb shell "ps -aef | grep px4 | grep -v grep")
   if [ -z "${checkProc}" ]; then
      echo "[ERROR] PX4 is not running on target!"
      exit $EXIT_ERROR
   fi

   # Stop the PX4 process on target
   adb shell "ps -eaf | grep px4 | grep -v grep | awk '{print $2}' | tr -s ' ' | cut -d' ' -f2 | xargs kill"
   sleep 5

   # Stop the mini-dm
   killall mini-dm

   echo -e "PX4 test complete."

   # Verify the results
   verifypx4test

   echo -e "For more information, see px4.log and minidm.log."
}


usage() {
  echo -e "\nThis script can copy PX4 binaries to the Snapdragon Flight target and do a quick on-target sanity test.\n"
  echo -e "Pre-requisites:"
  echo -e "- Snapdragon Flight board must be connected to host computer via USB"
  echo -e "- Snapdragon Flight board must have the latest platform BSP and flight controller addon installed"
  echo -e "- mini-dm must be installed on host computer (see https://github.com/ATLFlight/ATLFlightDocs/blob/master/UserGuide.md#adsp)"
  echo -e "- PX4 software was built and binaries are in their usual locations in the tree\n"
  echo -e "USAGE:\n ${0} [-m mode] [-i] [-t] [-l <minidm-path>]"
  echo -e "    -m --> Build mode (0 = default mode, 1 = legacy mode)"
  echo -e "    -i --> Install the PX4 binaries"
  echo -e "    -t --> Test PX4 on target"
  echo -e "    -l --> location of the mini-dm executable (Default: ${minidmPath})"
  echo -e "    -h --> Display this help information"
}

# Parse the command line options
while getopts "m:l:ith" opt;
   do
      case $opt in
         m)
            if [ $OPTARG -gt $MODE_MAX ]; then
               echo "Invalid mode: $OPTARG (max allowed is $MODE_MAX)"
               exit $EXIT_ERROR
            fi
            mode=$OPTARG
            echo "Will run the script in mode $mode."
            ;;
         i)
            install=1
            ;;
         t)
            test=1
            ;;
         l)
            minidmPath=$OPTARG
            ;;
         h)
            usage
            exit 0
            ;;
         :)
            echo "Option -$OPTARG requires an argument" >&2
            exit 1;;
         ?)
            echo "Unknown arg $opt"
            usage
            exit 1
            ;;
      esac
done

# Install the PX4 binaries
installpx4

# Run the sanity test
testpx4

exit $result

