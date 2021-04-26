# LED control: control a single or multiple LED's.
# These are the externally visible LED's, not the board LED's

uint64 timestamp		# time since system start (microseconds)

# colors
uint8 COLOR_OFF = 0 # this is only used in the drivers
uint8 COLOR_RED = 1
uint8 COLOR_GREEN = 2
uint8 COLOR_BLUE = 3
uint8 COLOR_YELLOW = 4
uint8 COLOR_PURPLE = 5
uint8 COLOR_AMBER = 6
uint8 COLOR_CYAN = 7
uint8 COLOR_WHITE = 8

# LED modes definitions
uint8 MODE_OFF = 0 # turn LED off
uint8 MODE_ON = 1  # turn LED on
uint8 MODE_DISABLED = 2  # disable this priority (switch to lower priority setting)
uint8 MODE_BLINK_SLOW = 3
uint8 MODE_BLINK_NORMAL = 4
uint8 MODE_BLINK_FAST = 5
uint8 MODE_BREATHE = 6 # continuously increase & decrease brightness (solid color if driver does not support it)
uint8 MODE_FLASH = 7 # two fast blinks (on/off) with timing as in MODE_BLINK_FAST and then off for a while

uint8 MAX_PRIORITY = 2 # maxium priority (minimum is 0)


uint8 led_mask # bitmask which LED(s) to control, set to 0xff for all
uint8 color # see COLOR_*
uint8 mode # see MODE_*
uint8 num_blinks # how many times to blink (number of on-off cycles if mode is one of MODE_BLINK_*) . Set to 0 for infinite
                 # in MODE_FLASH it is the number of cycles. Max number of blinks: 122 and max number of flash cycles: 20
uint8 priority # priority: higher priority events will override current lower priority events (see MAX_PRIORITY)

uint8 ORB_QUEUE_LENGTH = 8      # needs to match BOARD_MAX_LEDS
