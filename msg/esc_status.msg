uint64 timestamp					# time since system start (microseconds)
uint8 CONNECTED_ESC_MAX = 8				# The number of ESCs supported. Current (Q2/2013) we support 8 ESCs

uint8 ESC_CONNECTION_TYPE_PPM = 0			# Traditional PPM ESC
uint8 ESC_CONNECTION_TYPE_SERIAL = 1			# Serial Bus connected ESC
uint8 ESC_CONNECTION_TYPE_ONESHOT = 2			# One Shot PPM
uint8 ESC_CONNECTION_TYPE_I2C = 3			# I2C
uint8 ESC_CONNECTION_TYPE_CAN = 4			# CAN-Bus
uint8 ESC_CONNECTION_TYPE_DSHOT = 5			# DShot

uint16 counter  					# incremented by the writing thread everytime new data is stored

uint8 esc_count						# number of connected ESCs
uint8 esc_connectiontype				# how ESCs connected to the system

uint8 esc_online_flags					# Bitmask indicating which ESC is online/offline
# esc_online_flags bit 0 : Set to 1 if ESC0 is online
# esc_online_flags bit 1 : Set to 1 if ESC1 is online
# esc_online_flags bit 2 : Set to 1 if ESC2 is online
# esc_online_flags bit 3 : Set to 1 if ESC3 is online
# esc_online_flags bit 4 : Set to 1 if ESC4 is online
# esc_online_flags bit 5 : Set to 1 if ESC5 is online
# esc_online_flags bit 6 : Set to 1 if ESC6 is online
# esc_online_flags bit 7 : Set to 1 if ESC7 is online

uint8 esc_armed_flags					# Bitmask indicating which ESC is armed. For ESC's where the arming state is not known (returned by the ESC), the arming bits should always be set.

esc_report[8] esc
