uint64 timestamp						# time since system start (microseconds)
uint8 NUM_ACTUATORS_MAX				= 8
uint32 noutputs							# valid outputs
float32[16] position						# actuator position normalized -1.0 to +1.0
uint8[16] power_percent						# percent of full power from 0 to 1

uint8 ACTUATOR_VENDOR_GENERIC 			= 0		# generic smart actuator
uint8 ACTUATOR_VENDOR_DYNAMIXEL 		= 1		# smart Dynamixel via RS485 or hardware CAN bridge

uint8 ACTUATOR_CONNECTION_TYPE_RS232 		= 0
uint8 ACTUATOR_CONNECTION_TYPE_RS485 		= 1
uint8 ACTUATOR_CONNECTION_TYPE_CAN 		= 2		# CAN-Bus

uint16 counter  						# incremented by the writing thread everytime new data is stored

uint8 actuator_connectiontype					# how actuators are connected to the system
