uint8 channel_index
uint16 update_mask
# Masks
uint16 MASK_WAVEFORM    = 1
uint16 MASK_FREQUENCY   = 2
uint16 MASK_AMPLITUDE   = 4
uint16 MASK_OFFSET      = 8
uint16 MASK_PHASE       = 16
uint16 MASK_OUTPUT_EN   = 32
uint16 MASK_OUTPUT_SYNC = 64
uint16 MASK_DUTY_CYCLE  = 128

uint8 waveform
float64 frequency
float32 amplitude
float32 offset
float32 phase
float32 duty_cycle
bool output_en
bool output_sync    # Only effective when channel_index is 1
