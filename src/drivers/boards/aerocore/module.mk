#
# Board-specific startup code for the AeroCore
#

SRCS		 = aerocore_init.c \
		   aerocore_pwm_servo.c \
		   aerocore_spi.c \
		   aerocore_led.c \
		   ../../../modules/systemlib/up_cxxinitialize.c

MAXOPTIMIZATION	 = -Os
