#
# LSM303D accel/mag driver
#

MODULE_COMMAND	 = lsm303d
SRCS		 = lsm303d.cpp

MODULE_STACKSIZE	= 1200

EXTRACXXFLAGS	= -Weffc++

MAXOPTIMIZATION	 = -Os
