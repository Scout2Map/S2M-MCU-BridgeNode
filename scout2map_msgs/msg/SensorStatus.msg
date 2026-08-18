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

# Sensor presence. Seeded from the MCU boot line, then confirmed by data:
# any sensor that delivers a reading is marked present even if the boot line
# was missed, which happens whenever this node attaches to an MCU that is
# already running. A boot line resets all four flags.
#
# These say a sensor is talking, not that its reading is usable. Use the
# matching *_valid fields in EnvSnapshot for that.
bool    aht21_present
bool    ens160_present
bool    bh1750_present
bool    pms7003_seen
