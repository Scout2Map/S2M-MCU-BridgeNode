# ENS160 gas / air quality reading from the Pico 2 sensor fusion MCU.

std_msgs/Header header

uint16 eco2_ppm         # equivalent CO2, 400 - 65000 ppm
uint16 tvoc_ppb         # total VOC, 0 - 65000 ppb
uint8  aqi              # UBA air quality index, 1 (excellent) - 5 (unhealthy)

# ENS160 DATA_STATUS validity flag, forwarded untouched from the MCU
uint8 VALIDITY_NORMAL=0         # normal operation, values are trustworthy
uint8 VALIDITY_WARMUP=1         # warm-up phase, values still settling
uint8 VALIDITY_INITIAL_START=2  # initial start-up phase (first ~1h of a new part)
uint8 VALIDITY_INVALID=3        # invalid output, discard the values
uint8 validity

bool operational        # true only when validity == VALIDITY_NORMAL
