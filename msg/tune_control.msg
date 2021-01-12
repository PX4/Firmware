# This message is used to control the tunes, when the tune_id is set to CUSTOM
# then the frequency, duration are used otherwise those values are ignored.

uint64 timestamp     # time since system start (microseconds)

uint8 TUNE_ID_STOP                 = 0
uint8 TUNE_ID_STARTUP              = 1
uint8 TUNE_ID_ERROR                = 2
uint8 TUNE_ID_NOTIFY_POSITIVE      = 3
uint8 TUNE_ID_NOTIFY_NEUTRAL       = 4
uint8 TUNE_ID_NOTIFY_NEGATIVE      = 5
uint8 TUNE_ID_ARMING_WARNING       = 6
uint8 TUNE_ID_BATTERY_WARNING_SLOW = 7
uint8 TUNE_ID_BATTERY_WARNING_FAST = 8
uint8 TUNE_ID_GPS_WARNING          = 9
uint8 TUNE_ID_ARMING_FAILURE       = 10
uint8 TUNE_ID_PARACHUTE_RELEASE    = 11
uint8 TUNE_ID_SINGLE_BEEP          = 12
uint8 TUNE_ID_HOME_SET             = 13
uint8 TUNE_ID_SD_INIT              = 14
uint8 TUNE_ID_SD_ERROR             = 15
uint8 TUNE_ID_PROG_PX4IO           = 16
uint8 TUNE_ID_PROG_PX4IO_OK        = 17
uint8 TUNE_ID_PROG_PX4IO_ERR       = 18
uint8 NUMBER_OF_TUNES              = 19

uint8 tune_id        # tune_id corresponding to TuneID::* from the tune_defaults.h in the tunes library
bool tune_override   # if true the tune which is playing will be stopped and the new started
uint16 frequency     # in Hz
uint32 duration      # in us
uint32 silence       # in us
uint8 volume         # value between 0-100 if supported by backend

uint8 VOLUME_LEVEL_MIN = 0
uint8 VOLUME_LEVEL_DEFAULT = 40
uint8 VOLUME_LEVEL_MAX = 100

uint8 ORB_QUEUE_LENGTH = 4
