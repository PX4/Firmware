#! /usr/bin/env bash

## Bash script to setup PX4 development environment on Arch Linux.
## Tested on Manjaro 18.0.1.
##
## Installs:
## - Common dependencies and tools for nuttx, jMAVSim
## - NuttX toolchain (omit with arg: --no-nuttx)
## - jMAVSim simulator (omit with arg: --no-sim-tools)
## - Gazebo simulator (not by default, use --gazebo)
##
## Not Installs:
## - FastRTPS and FastCDR

INSTALL_NUTTX="true"
INSTALL_SIM="true"
INSTALL_GAZEBO="false"

# Parse arguments
for arg in "$@"
do
	if [[ $arg == "--no-nuttx" ]]; then
		INSTALL_NUTTX="false"
	fi

	if [[ $arg == "--no-sim-tools" ]]; then
		INSTALL_SIM="false"
	fi

	if [[ $arg == "--gazebo" ]]; then
		INSTALL_GAZEBO="true"
	fi
done

# script directory
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

# check requirements.txt exists (script not run in source tree)
REQUIREMENTS_FILE="requirements.txt"
if [[ ! -f "${DIR}/${REQUIREMENTS_FILE}" ]]; then
	echo "FAILED: ${REQUIREMENTS_FILE} needed in same directory as arch.sh (${DIR})."
	return 1
fi

echo
echo "Installing PX4 general dependencies"

sudo pacman -Sy --noconfirm --needed \
	astyle \
	base-devel \
	ccache \
	clang \
	cmake \
	cppcheck \
	doxygen \
	gdb \
	git \
	gnutls \
	nettle \
	ninja \
	python-pip \
	rsync \
	shellcheck \
	tar \
	unzip \
	wget \
	zip \
	;

# Python dependencies
echo "Installing PX4 Python3 dependencies"
pip install --user -r ${DIR}/requirements.txt

# NuttX toolchain (arm-none-eabi-gcc)
if [[ $INSTALL_NUTTX == "true" ]]; then
	echo
	echo "Installing NuttX dependencies"

	sudo pacman -S --noconfirm --needed \
		gperf \
		vim \
		;

	if [ ! -z "$USER" ]; then
		# add user to dialout group (serial port access)
		sudo usermod -aG uucp $USER
	fi

	# remove modem manager (interferes with PX4 serial port usage)
	sudo pacman -R modemmanager --noconfirm

	# arm-none-eabi-gcc
	NUTTX_GCC_VERSION="7-2017-q4-major"
	GCC_VER_STR=$(arm-none-eabi-gcc --version)
	STATUSRETVAL=$(echo $GCC_VER_STR | grep -c "${NUTTX_GCC_VERSION}")

	if [ $STATUSRETVAL -eq "1" ]; then
		echo "arm-none-eabi-gcc-${NUTTX_GCC_VERSION} found, skipping installation"
	else
		echo "Installing arm-none-eabi-gcc-${NUTTX_GCC_VERSION}";
		wget -O /tmp/gcc-arm-none-eabi-${NUTTX_GCC_VERSION}-linux.tar.bz2 https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/7-2017q4/gcc-arm-none-eabi-${NUTTX_GCC_VERSION}-linux.tar.bz2 && \
			sudo tar -jxf /tmp/gcc-arm-none-eabi-${NUTTX_GCC_VERSION}-linux.tar.bz2 -C /opt/;

		# add arm-none-eabi-gcc to user's PATH
		exportline="export PATH=/opt/gcc-arm-none-eabi-${NUTTX_GCC_VERSION}/bin:\$PATH"

		if grep -Fxq "$exportline" $HOME/.profile;
		then
			echo "${NUTTX_GCC_VERSION} path already set.";
		else
			echo $exportline >> $HOME/.profile;
		fi
	fi
fi

# Simulation tools
if [[ $INSTALL_SIM == "true" ]]; then
	echo
	echo "Installing PX4 simulation dependencies"

	# java (jmavsim or fastrtps)
	sudo pacman -S --noconfirm --needed \
		ant \
		jdk-openjdk \
		;

	# Gazebo setup
	if [[ $INSTALL_GAZEBO == "true" ]]; then
		echo
		echo "Installing gazebo and dependencies for PX4 gazebo simulation"

		# PX4 gazebo simulation dependencies
		sudo pacman -S --noconfirm --needed \
			dmidecode \
			eigen3 \
			hdf5 \
			opencv \
			protobuf \
			vtk \
			;

		# add community binary repository for gazebo and ROS
		# https://wiki.archlinux.org/index.php/Unofficial_user_repositories#oscloud
		if ! grep -q oscloud /etc/pacman.conf; then
			echo "# ROS gazebo repository for PX4
[oscloud]
SigLevel = Never
Server = http://repo.oscloud.info/" | sudo tee -a /etc/pacman.conf > /dev/null
		fi

		sudo pacman -Sy --noconfirm --needed gazebo

		if sudo dmidecode -t system | grep -q "Manufacturer: VMware, Inc." ; then
			# fix VMWare 3D graphics acceleration for gazebo
			echo "export SVGA_VGPU10=0" >> ~/.profile
		fi
	fi
fi

if [[ $INSTALL_NUTTX == "true" ]]; then
	echo
	echo "Reboot or logout, login computer before attempting to build NuttX targets"
fi
