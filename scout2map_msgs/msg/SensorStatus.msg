# Health of the Pico 2 serial link and of the bridge node itself.

std_msgs/Header header

string  port                # serial device actually opened
bool    port_open
bool    link_ok             # port open and a line arrived within the timeout
float32 last_line_age_s

uint32  mcu_uptime_ms       # from the MCU heartbeat line
uint32  mcu_reboot_count    # incremented whenever uptime jumps backwards

uint64  lines_received
uint64  parse_errors        # lines that were not valid JSON
uint64  unknown_src         # valid JSON but an unrecognised "src" field
uint64  framing_overflows   # runaway bytes with no newline, buffer was reset

# Sensor presence as reported by the MCU boot line.
# pms7003 has no init handshake, so it is inferred from received frames.
bool    aht21_present
bool    ens160_present
bool    bh1750_present
bool    pms7003_seen
