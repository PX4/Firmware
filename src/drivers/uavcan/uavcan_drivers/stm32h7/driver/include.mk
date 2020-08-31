#
# Copyright (C) 2014 Pavel Kirienko <pavel.kirienko@gmail.com>
#

LIBUAVCAN_STM32H7_DIR := $(dir $(lastword $(MAKEFILE_LIST)))

LIBUAVCAN_STM32H7_SRC := $(shell find $(LIBUAVCAN_STM32H7_DIR)src -type f -name '*.cpp')

LIBUAVCAN_STM32H7_INC := $(LIBUAVCAN_STM32H7_DIR)include/
