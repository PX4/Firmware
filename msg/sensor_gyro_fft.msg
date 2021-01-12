uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id          # unique device ID for the sensor that does not change between power cycles

float32 dt                # delta time between samples (microseconds)

float32 resolution_hz

float32[3] peak_frequency # peak frequency per axis

float32[4] peak_frequencies_x # x axis peak frequencies
float32[4] peak_frequencies_y # y axis peak frequencies
float32[4] peak_frequencies_z # z axis peak frequencies
