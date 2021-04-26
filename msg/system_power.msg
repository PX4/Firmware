uint64 timestamp		# time since system start (microseconds)
float32 voltage5v_v		# peripheral 5V rail voltage
float32[4] sensors3v3		# Sensors 3V3 rail voltage
uint8 sensors3v3_valid		# Sensors 3V3 rail voltage was read (bitfield).
uint8 usb_connected		# USB is connected when 1
uint8 brick_valid		# brick bits power is good when bit 1
uint8 usb_valid 		# USB is valid when 1
uint8 servo_valid		# servo power is good when 1
uint8 periph_5v_oc		# peripheral overcurrent when 1
uint8 hipower_5v_oc		# high power peripheral overcurrent when 1
uint8 comp_5v_valid		# 5V to companion valid
uint8 can1_gps1_5v_valid	# 5V for CAN1/GPS1 valid

uint8 BRICK1_VALID_SHIFTS=0
uint8 BRICK1_VALID_MASK=1
uint8 BRICK2_VALID_SHIFTS=1
uint8 BRICK2_VALID_MASK=2
uint8 BRICK3_VALID_SHIFTS=2
uint8 BRICK3_VALID_MASK=4
uint8 BRICK4_VALID_SHIFTS=3
uint8 BRICK4_VALID_MASK=8
