uint64 timestamp               # time since system start (microseconds)

uint8 LOGGER_TYPE_FULL    = 0  # Normal, full size log
uint8 LOGGER_TYPE_MISSION = 1  # reduced mission log (e.g. for geotagging)
uint8 type

uint8 BACKEND_FILE    = 1
uint8 BACKEND_MAVLINK = 2
uint8 BACKEND_ALL     = 3
uint8 backend

float32 total_written_kb       # total written to log in kiloBytes
float32 write_rate_kb_s        # write rate in kiloBytes/s

uint32 dropouts                # number of failed buffer writes due to buffer overflow
uint32 message_gaps            # messages misssed

uint32 buffer_used_bytes       # current buffer fill in Bytes
uint32 buffer_size_bytes       # total buffer size in Bytes

uint8 num_messages
