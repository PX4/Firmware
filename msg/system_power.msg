uint64 timestamp		# time since system start (microseconds)
float32 voltage5v_v		# peripheral 5V rail voltage
float32 voltage3v3_v		# Sensor 3V3 rail voltage
uint8 v3v3_valid		# Sensor 3V3 rail voltage was read.
uint8 usb_connected		# USB is connected when 1
uint8 brick_valid		# brick bits power is good when bit 1
uint8 usb_valid 		# USB is valid when 1
uint8 servo_valid		# servo power is good when 1
uint8 periph_5v_oc		# peripheral overcurrent when 1
uint8 hipower_5v_oc		# high power peripheral overcurrent when 1

uint8 BRICK1_VALID_SHIFTS=0
uint8 BRICK1_VALID_MASK=1
uint8 BRICK2_VALID_SHIFTS=1
uint8 BRICK2_VALID_MASK=2
uint8 BRICK3_VALID_SHIFTS=2
uint8 BRICK3_VALID_MASK=4
uint8 BRICK4_VALID_SHIFTS=3
uint8 BRICK4_VALID_MASK=8
