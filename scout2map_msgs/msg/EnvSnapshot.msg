# Latest-value cache of every environment sensor, published at a fixed rate.
# Each sensor runs at its own period on the MCU, so this message carries the
# most recent value of each plus how old that value is. Consumers that need
# "all sensors at this pose" should use this instead of joining raw topics.

std_msgs/Header header

# --- AHT21 (temperature / humidity) ---
float32 temperature_c
float32 humidity_pct        # 0 - 100 %, NOT the 0 - 1 ratio used by sensor_msgs
bool    ambient_valid       # false when the reading is missing or stale
float32 ambient_age_s

# --- BH1750 (ambient light) ---
float32 illuminance_lux
bool    illuminance_valid
float32 illuminance_age_s

# --- ENS160 (gas / air quality) ---
uint16  eco2_ppm
uint16  tvoc_ppb
uint8   aqi
uint8   ens160_validity     # see AirQuality.msg for the meaning of each value
bool    air_quality_valid   # fresh AND validity == 0
float32 air_quality_age_s

# --- PMS7003 (particulate matter) ---
uint16  pm1_0_ug_m3
uint16  pm2_5_ug_m3
uint16  pm10_ug_m3
bool    particulate_valid
float32 particulate_age_s

# --- Link health ---
bool    link_ok             # serial port open and lines arriving
uint32  mcu_uptime_ms       # uptime reported by the MCU heartbeat
