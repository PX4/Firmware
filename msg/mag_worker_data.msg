uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample

uint8 MAX_MAGS = 4

uint32 done_count
uint32 calibration_points_perside
uint64 calibration_interval_perside_us
uint32[4] calibration_counter_total
bool[4] side_data_collected
float32[4] x
float32[4] y
float32[4] z
