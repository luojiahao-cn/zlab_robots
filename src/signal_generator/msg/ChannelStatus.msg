uint8 channel_index     # 1, 2, or 3
uint8 waveform         # 0-99, numeric code
float64 frequency      # Hz
float32 amplitude      # V
float32 offset         # V
float32 phase          # degrees
float32 duty_cycle     # percent
bool output_enabled    # true=ON, false=OFF
bool valid             # false if read failed (stale data)
