# scout2map_bridge

두 MCU를 ROS2에 연결하는 브릿지 노드들이다.

| 노드 | 대상 | 방향 | 프레이밍 | 네임스페이스 |
|---|---|---|---|---|
| `sensor_bridge` | 센서 퓨전 MCU (Pico 2) | 수신 전용 | 개행 구분 JSON | `/sensors/*` |
| `drive_bridge` | 주행 제어 MCU (STM32) | **양방향** | 길이 접두 바이너리 + CRC16 | `/drive/*` |
| `fake_sensors` | 없음 (합성 데이터) | - | - | `/sensors/*` |

이 문서는 **노드 자체를 다루거나 고치는 사람**을 위한 것이다.
토픽에 어떤 값이 들어오는지 알고 싶을 뿐이라면
[`scout2map_msgs/README.md`](../scout2map_msgs/README.md)를 본다.
STM32 와이어 포맷은 [`PROTOCOL.md`](PROTOCOL.md)에 있다.

두 노드가 `serial_link.py`의 `SerialLink`를 공유한다. 포트 개방, 재연결,
리더 스레드 관리가 여기에 있고, **프레이밍은 공유하지 않는다.**
두 MCU가 합의하지 않기 때문이다.

```
SerialLink  (포트 소유, 원시 바이트)
   ├── LineFramer      -> sensor_bridge   개행 분할
   └── FrameDecoder    -> drive_bridge    SOF 탐색 + 길이 + CRC16
```

`LineFramer`도 `serial_link.py`에 있고, `FrameDecoder`는 프로토콜에
종속적이므로 `drive_protocol.py`에 있다.

---

# 1부. sensor_bridge

## 1. MCU 입력 포맷

Pico 2 펌웨어는 JSON 객체 한 개를 한 줄로 내보낸다. `src` 필드가 페이로드 종류를 결정한다.

```json
{"src":"sys","event":"boot","aht21":true,"ens160":true,"bh1750":true}
{"src":"bh1750","lux":123.4}
{"src":"aht21","temp":25.31,"hum":41.02}
{"src":"ens160","eco2":412,"tvoc":37,"aqi":1,"valid":0}
{"src":"pms7003","pm1":3,"pm25":5,"pm10":6}
{"src":"sys","uptime_ms":123456}
```

센서마다 독립된 주기로 발행되므로 라인 순서는 보장되지 않는다.
펌웨어 측은 스핀락으로 보호된 링 버퍼(`line_queue`)를 두어 라인이 섞이지 않도록 한다.

---

## 2. 노드 구조

```
[USB CDC] ──▶ SerialLink (별도 스레드, 원시 바이트)
                   │  LineFramer가 \n 단위로 잘라 (수신시각, 라인) 생성
                   ▼
              deque (maxlen 512)
                   │
                   ▼
              _drain_rx (ROS 타이머 100Hz)
                   │  JSON 파싱 → src 라우팅
                   ├──▶ 개별 토픽 즉시 발행
                   └──▶ 최신값 캐시 갱신
                              │
                              ├──▶ _publish_snapshot (5Hz)
                              └──▶ _publish_status (1Hz)
```

**시리얼 읽기만 별도 스레드**이고 발행은 전부 ROS 실행 스레드에서 일어난다.
퍼블리셔 동시 접근 문제를 구조적으로 피하기 위한 설계이며,
큐 경유로 생기는 지연은 최대 10ms 수준이라 무시할 만하다.

deque가 가득 차면 **가장 오래된 라인부터 버린다.** ROS 측이 잠시 멈추더라도
메모리가 무한히 늘지 않는 쪽을 택했다.

### 최신값 캐시

센서별 마지막 값과 그 수신 시각(monotonic)을 들고 있다가, 스냅샷 발행 시점에
나이를 계산해 유효 플래그와 함께 내보낸다. monotonic 시계를 쓰므로
시스템 시각이 NTP로 점프해도 나이 계산이 어긋나지 않는다.

---

## 3. 파라미터

| 이름 | 기본값 | 설명 |
|---|---|---|
| `port` | `/dev/scout2map_pico` | 시리얼 장치 경로 |
| `baudrate` | `115200` | USB CDC라 실제로는 무시되지만 pyserial이 요구한다 |
| `frame_id` | `sensor_fusion` | 모든 메시지의 `header.frame_id` |
| `snapshot_rate_hz` | `5.0` | 스냅샷 발행 주기 |
| `status_rate_hz` | `1.0` | 상태 발행 주기 |
| `publish_raw_json` | `false` | 원본 라인을 `/sensors/raw_json`에 미러링 |
| `stale_ambient_s` | `3.0` | AHT21 값의 유효 수명 |
| `stale_illuminance_s` | `1.0` | BH1750 값의 유효 수명 |
| `stale_air_quality_s` | `3.0` | ENS160 값의 유효 수명 |
| `stale_particulate_s` | `5.0` | PMS7003 값의 유효 수명 |
| `link_timeout_s` | `3.0` | 이 시간 동안 라인이 없으면 링크 다운으로 본다 |

`stale_*` 값은 각 센서의 발행 주기보다 넉넉히 크게 잡아야 한다.
1Hz 센서에 1.0초를 주면 지터 한 번에 `valid` 플래그가 깜빡거린다.

값은 `config/sensor_bridge.yaml`에서 수정하는 것이 기본이다.
런치 파일이 이 YAML을 읽어 노드에 넘긴다.

일회성으로 바꿔 볼 때는 `ros2 run`에 직접 넘긴다.
다만 이 경우 YAML은 읽히지 않으므로, 명시하지 않은 값은 코드의 기본값이 쓰인다.

```bash
ros2 run scout2map_bridge sensor_bridge --ros-args \
  -p port:=/dev/ttyACM0 \
  -p publish_raw_json:=true
```

현재 적용된 값은 실행 중에 조회할 수 있다.

```bash
ros2 param list /sensor_bridge
ros2 param get /sensor_bridge stale_air_quality_s
```

설치와 빌드 절차는 [레포 루트 README](../README.md)에 있다.

---

## 4. QoS

| 토픽 | 신뢰성 | 내구성 | depth |
|---|---|---|---|
| `/sensors/*` | RELIABLE | VOLATILE | 10 |
| `/sensors/status` | RELIABLE | TRANSIENT_LOCAL | 1 |

센서 데이터에는 BEST_EFFORT를 쓰는 관례가 있지만 여기서는 RELIABLE을 택했다.
데이터 레이트가 초당 수 건 수준이라 신뢰성 전송 비용이 사실상 없고,
**이벤트 엔진이 임계값 돌파 순간의 샘플을 조용히 놓치는 쪽이 훨씬 위험**하기 때문이다.

`/sensors/status`만 TRANSIENT_LOCAL이라 나중에 뜬 노드도 마지막 상태를 즉시 받는다.

---

## 5. 견고성 설계

- **자동 재연결.** USB가 빠지거나 MCU가 리셋되면 1초 간격으로 포트 재개방을 시도한다.
  노드를 다시 띄울 필요가 없다.
- **MCU 리부트 감지.** 하트비트의 `uptime_ms`가 뒤로 점프하면 리부트로 간주해
  `mcu_reboot_count`를 올리고 `pms7003_seen`을 false로 되돌린다.
  전원 계통 문제를 추적할 때 쓴다.
- **깨진 라인은 노드를 죽이지 않는다.** JSON 파싱 실패, 필드 누락, NaN/inf는
  카운터만 올리고 폐기한다. 로그는 50건마다 한 번씩만 남겨 콘솔 폭주를 막는다.
- **범위 클램프.** MCU 값이 이상해도 `uint16`/`uint8` 범위로 잘라 넣으므로
  메시지 직렬화 단계에서 예외가 나지 않는다.
- **버퍼 오버플로 방어.** 개행 없이 4KB가 쌓이면 스트림이 깨진 것으로 보고 버퍼를 비운다.

---

## 6. 트러블슈팅

먼저 상태부터 본다.

```bash
ros2 topic echo /sensors/status --once
```

| 증상 | 원인과 조치 |
|---|---|
| `port_open: false` | 장치 경로가 틀렸거나 권한이 없다. `ls -l /dev/ttyACM*` 확인 후 `dialout` 그룹 가입 여부를 본다 |
| `port_open: true`인데 `link_ok: false` | 포트는 열렸으나 라인이 없다. MCU 펌웨어가 안 돌거나 USB 케이블이 데이터선 없는 충전 전용일 수 있다 |
| `last_line_age_s: -1.0` | 아직 한 줄도 못 받았다는 특수값이다. 위와 동일하게 확인한다 |
| `parse_errors`만 계속 증가 | 보드레이트나 포트를 잘못 잡아 다른 장치를 읽고 있을 가능성이 크다. `publish_raw_json:=true`로 켜고 실제 내용을 본다 |
| `unknown_src` 증가 | 펌웨어에 센서를 추가하고 브릿지 라우팅을 안 고쳤다. 7절 참조 |
| `*_present`가 false인데 `lines_received`는 증가 | 초기화 실패가 아니라 부팅 라인을 놓친 것이다. MCU를 리셋해 다시 본다 |
| `mcu_reboot_count`가 계속 증가 | MCU가 반복 리셋되고 있다. 전원 계통이나 PMS7003의 VBUS 부하를 의심한다 |

원본 라인을 직접 보는 것이 가장 빠른 진단이다.

```bash
ros2 run scout2map_bridge sensor_bridge --ros-args -p publish_raw_json:=true
ros2 topic echo /sensors/raw_json
```

---

## 7. 펌웨어에 센서를 추가했을 때

브릿지도 함께 고쳐야 한다. 라우팅을 등록하지 않으면 `unknown_src`만 올라가고
데이터는 조용히 버려진다.

1. `_handle_line`의 `src` 분기에 새 항목을 추가한다.
2. `_on_<sensor>` 핸들러를 만들어 개별 토픽 발행과 캐시 갱신을 넣는다.
3. `self._cache` 딕셔너리에 키를 추가한다.
4. 스냅샷에 포함할 값이면 `EnvSnapshot.msg`에 필드를 추가하고 `_publish_snapshot`을 고친다.
5. 유효 수명 파라미터(`stale_*`)를 추가한다.

---

## 8. 알려진 제약

- **역방향 통신이 없다.** 현재 Pico 2 펌웨어는 수신 명령을 처리하지 않으므로
  이 노드는 송신하지 않는다. `SerialLink.write()`는 이미 있으므로
  (주행 브릿지가 쓴다) 필요해지면 펌웨어에 파서를 넣는 쪽이 작업량이다.
- **`SerialReader`가 이 파일 안에 있다.** 포트 개방·재연결 로직 자체는 범용이지만
  아직 공용 모듈로 분리하지 않았다. STM32 브릿지는 프레이밍 방식이 다르므로
  (개행 분할이 아니라 SOF 탐색 + CRC16 검증) 그때 함께 정리한다.
- **`stamp`은 수신 시각이다.** MCU 측정 시각이 아니다. 자세한 내용은 msg 문서 2절 참조.

---

## 9. 가짜 데이터 퍼블리셔 (fake_sensors)

하드웨어 없이 구독자 측을 개발할 수 있도록 `fake_sensor_node.py`를 함께 둔다.
브릿지와 동일한 토픽·타입·주기로 합성 값을 발행하며, 시나리오로 값의 흐름을 바꾼다.
사용법은 [레포 루트 README 6절](../README.md#6-하드웨어-없이-개발하기)에 정리되어 있다.

브릿지를 고칠 때도 쓸모가 있다. 토픽 계약을 바꿨다면 이 노드도 함께 고쳐야
가짜 데이터와 진짜 데이터가 계속 같은 모양을 유지한다.
특히 `EnvSnapshot`에 필드를 추가했다면 `_tick_snapshot` 양쪽을 모두 수정한다.

유효 수명 판정 기준(3초, 1초, 5초)이 이 노드에는 상수로 박혀 있다.
브릿지 쪽 `stale_*` 파라미터의 기본값과 맞춰 둔 것이므로,
기본값을 바꾸면 이 노드의 상수도 같이 옮긴다.

---

# 2부. drive_bridge

STM32 주행 제어 MCU를 ROS2에 연결한다.
와이어 포맷 상세는 [`PROTOCOL.md`](PROTOCOL.md)에 있고, 여기서는 노드 동작을 다룬다.

## 10. sensor_bridge와 무엇이 다른가

| | `sensor_bridge` | `drive_bridge` |
|---|---|---|
| 방향 | 수신 전용 | **양방향** |
| 프레이밍 | 개행 구분 JSON | 길이 접두 바이너리 + CRC16 |
| 유실 프레임 | 다음 주기에 자연 복구 | 동일하나 **명령은 워치독이 걸림** |
| 실패 시 결과 | 센서값 결측 | **로봇이 계속 움직임** |

마지막 행이 설계 전반을 좌우한다. 센서 브릿지가 멈추면 데이터가 비지만,
주행 브릿지가 잘못 멈추면 차체가 마지막 명령대로 계속 굴러간다.

## 11. 명령 경로와 두 겹의 워치독

```
/cmd_vel (Twist)
   │  구독 콜백이 값만 캐시한다
   ▼
[캐시: linear, angular, 수신 시각]
   │
   ▼ 20Hz 타이머
   ├── 최근 0.25초 안에 갱신됨 → 캐시값 전송
   └── 그보다 오래됨          → 명시적 0 전송
                                  │
                                  ▼
                          [STM32: 300ms 무명령 시 자체 정지]
```

**왜 캐시하고 반복 전송하는가.** MCU는 300ms 동안 명령이 없으면 모터를
멈춘다. `/cmd_vel` 발행자가 5Hz로 도는 경우 프레임 하나만 유실돼도
400ms 공백이 생겨 로봇이 덜컥거린다. 20Hz로 반복 전송하면 이 문제가 없다.

**왜 브릿지가 먼저 0을 보내는가.** 반복 전송만 있으면 `/cmd_vel` 발행 노드가
죽어도 브릿지가 마지막 명령을 영원히 되풀이한다. 그래서 브릿지 쪽에
더 짧은 타임아웃(0.25초)을 두고, 만료되면 스스로 정지를 명령한다.

**MCU의 300ms는 최후의 방어선이다.** 브릿지 자체가 죽거나 USB가 빠졌을 때를
위한 것이지 정상 경로가 아니다. 정상 경로에서는 브릿지가 먼저 0을 보낸다.

## 12. 발행 토픽

| 토픽 | 타입 | 비고 |
|---|---|---|
| `/drive/odom` | `nav_msgs/Odometry` | `publish_tf`가 참이면 `odom`→`base_link` TF도 함께 |
| `/drive/imu` | `sensor_msgs/Imu` | 각속도는 Z만 유효 |
| `/drive/range` | `sensor_msgs/Range` | 센티널 처리 주의 |
| `/drive/battery` | `sensor_msgs/BatteryState` | 미보고 시 `voltage=NaN` |
| `/drive/status` | `scout2map_msgs/DriveStatus` | 10Hz |
| `/drive/diagnostics` | `scout2map_msgs/DriveDiagnostics` | 요청 시에만 |

구독은 `/cmd_vel` (`geometry_msgs/Twist`) 하나다.
`linear.x`와 `angular.z`만 사용하며, 나머지 성분은 차동 구동 차체에서
의미가 없으므로 무시한다.

## 13. 서비스

모두 `std_srvs/Trigger`다.

```bash
ros2 service call /drive/estop std_srvs/srv/Trigger
ros2 service call /drive/clear_fault std_srvs/srv/Trigger
ros2 service call /drive/reset_odom std_srvs/srv/Trigger
ros2 service call /drive/request_diagnostics std_srvs/srv/Trigger
```

**E-stop은 래치된다.** 이후 속도 명령을 아무리 보내도 풀리지 않으며
`clear_fault`를 명시적으로 호출해야 복귀한다. 브릿지도 e-stop 요청 시
캐시된 명령을 0으로 지우므로, 복귀에는 의도적인 행동이 두 번 필요하다.

## 14. 파라미터

| 이름 | 기본값 | 설명 |
|---|---|---|
| `port` | `/dev/scout2map_drive` | udev 심볼릭 링크 |
| `command_rate_hz` | `20.0` | 명령 반복 주기 |
| `command_timeout_s` | `0.25` | 이보다 오래 `/cmd_vel`이 없으면 정지 명령 |
| `max_linear_mps` | `0.20` | 정격 58RPM 기준. 무부하 76RPM 아님 |
| `max_angular_radps` | `0.80` | 기구 한계 1.67보다 낮게. 스캔 품질 때문 |
| `publish_tf` | `true` | `odom`→`base_link` 발행 여부 |
| `odom_frame` / `base_frame` | `odom` / `base_link` | |
| `imu_frame` / `range_frame` | `imu_link` / `range_link` | |
| `link_timeout_s` | `0.5` | 텔레메트리가 50Hz이므로 넉넉한 값 |
| `range_min_m` / `range_max_m` | `0.04` / `0.30` | 아날로그 IR 기준 |
| `range_fov_rad` | `0.14` | |
| `default_track_width_m` | `0.24` | `BOOT_INFO` 도착 전까지만 사용 |
| `odom_xy_variance` | `0.001` | |
| `odom_yaw_variance` | `0.01` | |
| `odom_openloop_multiplier` | `100.0` | 개루프 시 공분산 배수 |

`max_linear_mps`를 0.262(무부하 76RPM)로 올리면 안 된다. 실제로 도달할 수
없는 값이라 제어기가 영구 포화 상태에 놓인다.

VL53L0X로 교체한 경우 `range_min_m: 0.03`, `range_max_m: 1.20`으로 바꾼다.

**`BOOT_INFO`가 파라미터를 덮어쓴다.** 트랙 폭은 펌웨어가 보고하는 값이
우선이다. 펌웨어는 실제 빌드를 알고 파라미터는 추정값이기 때문이며,
모터를 교체해도 브릿지가 자동으로 따라온다.

## 15. TF를 누가 발행하는가

기본값은 `drive_bridge`가 `odom`→`base_link`를 발행하는 것이다.
`robot_localization` 등으로 IMU와 엔코더를 융합할 계획이라면
`publish_tf: false`로 내리고 그쪽에 맡긴다.

**두 노드가 같은 TF를 발행하면 프레임이 떨린다.** 증상이 SLAM 품질 저하로만
나타나서 원인 찾기가 어려우므로, 융합 노드를 도입하는 시점에 반드시 끈다.

## 16. 오도메트리 신뢰도

펌웨어가 적분한 자세를 그대로 발행하고, 속도는 좌우 휠 속도에서 계산한다.
공분산은 상태에 따라 달라진다.

| 상태 | x/y 분산 | yaw 분산 |
|---|---|---|
| 정상 (폐루프) | 0.001 | 0.01 |
| `OPENLOOP` | 0.1 | 1.0 |

엔코더 신호가 끊기면 MCU가 개루프로 폴백하는데, 이때 오도메트리는 명령값
기반 추정이라 사실상 믿을 수 없다. 공분산을 100배로 부풀려 하위 필터가
스스로 알아내도록 방치하지 않는다.

`Imu`의 방위 공분산도 마찬가지로 `IMU_CALIBRATED` 비트에 따라 달라진다.
지자계 캘리브레이션 전에는 절대 방위가 드리프트하므로 yaw 분산을 1.0으로
둔다. 상대적 자세 변화는 그 전에도 쓸 수 있다.

## 17. 상태 변화 로깅

주행부 폴트는 50Hz로 계속 올라오므로, 매 프레임 찍으면 콘솔이 못 쓰게 된다.
브릿지는 **비트가 바뀌는 순간에만** 로그를 남긴다.

```
[ERROR] E-stop latched. Velocity commands are ignored until drive/clear_fault is called.
[ERROR] stall fault: high duty with no motion. Check for a jammed wheel before clearing.
[WARN]  encoder feedback lost, MCU fell back to open loop. Odometry is unreliable from here.
[ERROR] battery below the cell damage point. The MCU has cut drive. Land the robot and charge.
```

## 18. 트러블슈팅

```bash
ros2 topic echo /drive/status --once
```

| 증상 | 원인과 조치 |
|---|---|
| `link_ok: false`, `port_open`도 false | 장치 경로 또는 권한. udev 규칙과 `dialout` 그룹 확인 |
| `crc_errors`만 계속 증가 | 다른 장치를 열었을 가능성. Pico 2 포트를 잡았는지 확인 |
| `frames_ok`는 느는데 `estop_latched: true` | E-stop 래치 상태. `clear_fault` 호출 |
| 명령을 보내도 안 움직임 | `estop_latched`, `fault_stall`, `batt_dead` 순으로 확인 |
| `cmd_timeout: true` 반복 | `/cmd_vel` 발행이 끊기고 있다. 발행 노드 확인 |
| `openloop: true` | 엔코더 배선. 모터 전원선과 분리 포설했는지 확인 |
| `proto_version` 불일치 경고 | 펌웨어와 브릿지 버전 차이. 한쪽을 갱신 |
| `... frame is N bytes, this bridge expects M` | 페이로드 레이아웃 변경. `PROTOCOL.md` 2절 참조 |

IMU가 의심되면 진단 프레임을 요청한다.

```bash
ros2 service call /drive/request_diagnostics std_srvs/srv/Trigger
ros2 topic echo /drive/diagnostics --once
```

`imu_chip_id`가 0xA0이면 BNO055가 응답한 것이고, 0xFF면 전혀 응답이 없는
것이다. `i2c_recoveries`가 계속 증가하면 슬레이브가 SDA를 잡고 있다.

## 19. 검증 상태

`drive_protocol.py`는 펌웨어 저장소의 `tools/s2m_console.py`와 CRC 구현,
프레임 인코딩, 페이로드 포맷 문자열이 바이트 단위로 일치함을 확인했다.
디코더는 단편화·손상·노이즈 시나리오에서 복구를 검증했다.

**실물 STM32와의 통신은 아직 검증되지 않았다.** 위 확인은 두 호스트 측
구현이 서로 일치한다는 것이지, 실제 보드에서 프레임이 흐르는 것을
확인한 것이 아니다. 첫 브링업 시 20절 순서를 따른다.

## 20. 첫 브링업 순서

**차체를 들어 바퀴를 띄운 상태에서 시작한다.**

1. 펌웨어 저장소의 `./tools/s2m_console.py --ping`으로 먼저 링크를 확인한다.
   브릿지보다 이쪽이 문제 범위를 좁히기 쉽다.
2. 브릿지를 띄우고 `BOOT_INFO` 로그가 5764 counts/rev를 보고하는지 본다.
3. `ros2 topic echo /drive/status --once`로 `link_ok`와 `crc_errors`를 확인한다.
4. 바퀴를 손으로 돌려 `/drive/odom`의 자세가 변하는지 본다.
5. `ros2 topic pub`으로 아주 작은 속도를 명령하여 방향을 확인한다.
6. 마지막으로 바닥에 내린다.

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}}"
```

## 21. 알려진 제약

- **`CMD_WHEEL_RAW`를 노출하지 않는다.** PID를 우회하는 경로라 바닥에 놓인
  상태로 실행하면 즉시 주행한다. 브링업용이므로 펌웨어 저장소의
  `s2m_console.py --raw`를 쓰는 편이 안전하다.
- **`sensor_bridge`는 아직 `serial_link.py`를 쓰지 않는다.** 리더 로직이
  자체 구현으로 남아 있다. 실물 검증이 끝난 노드를 재검증 수단 없이
  건드리지 않기 위해 미뤘으며, 다음 하드웨어 세션에서 정리한다.
- **배터리 잔량이 `NaN`이다.** 이 팩의 방전 곡선을 특성화한 적이 없다.
  전압만으로 추정하면 모터 부하 아래에서 크게 틀린다.

## 22. 슬립 검출 신호

`DriveStatus`에 회전 슬립 신호를 싣는다. **판정이 아니라 신호다.**
임계값과 디바운스, 마커 생성은 이벤트 엔진 몫이며, 브릿지는 두 추정값의
불일치만 계산해 올린다. 튜닝이 두 저장소로 쪼개지지 않게 하기 위해서다.

| 필드 | 의미 |
|---|---|
| `yaw_rate_encoder_radps` | 좌우 휠 속도차로 계산한 요레이트 |
| `yaw_rate_imu_radps` | 자이로 실측에서 바이어스를 뺀 값 |
| `yaw_rate_error_radps` | 둘의 차이, 부호 있음 |
| `slip_ratio` | 정규화된 불일치. 접지 중이면 0에 가깝다 |
| `slip_signal_valid` | **false면 위 네 값을 전부 무시한다** |

두 추정은 완전히 독립적이다. 엔코더는 바퀴가 얼마나 돌았는지를 보고,
IMU는 차체가 실제로 얼마나 돌았는지를 본다. 접지 상태에서는 일치하고
미끄러지면 갈라진다. 이것이 슬립 이벤트의 전부다.

### 반드시 먼저 측정할 것 — `skid_factor`

**이 차체는 4륜 스키드 스티어다.** 차동 구동 2륜과 달리, 회전은 바퀴를
옆으로 끄는 방식으로 일어난다. 휠베이스가 140mm이므로 제자리 회전 시
네 바퀴 모두 반드시 미끄러진다. 구조적으로 그렇다.

따라서 기하학적 트랙폭 240mm로 계산하면 **실제보다 큰 요레이트가 나온다.**
`skid_factor`가 이를 보정하며, 타이어와 바닥재의 성질이므로 계산으로
구할 수 없고 실측해야 한다.

보정하지 않으면 어떻게 되는지가 이 표다. 동일한 정상 회전(바퀴 ±81mm/s,
차체 실제 0.5 rad/s)에 대한 결과다.

| `skid_factor` | `yaw_enc` | `yaw_imu` | `slip_ratio` | |
|---|---|---|---|---|
| 1.00 (미측정) | 0.675 | 0.500 | **0.26** | 정상 회전이 슬립으로 |
| 1.20 | 0.563 | 0.500 | 0.11 | |
| 1.35 (실측 가정) | 0.500 | 0.500 | **0.00** | 올바름 |
| 1.50 | 0.450 | 0.500 | 0.10 | |

진짜 슬립은 이렇게 나온다. 바퀴는 같은 속도로 도는데 차체가 못 따라오는 경우다.

| 차체 실제 회전 | `slip_ratio` |
|---|---|
| 0.50 rad/s (접지) | 0.00 |
| 0.35 rad/s | 0.30 |
| 0.20 rad/s | 0.60 |
| 0.05 rad/s (거의 헛돎) | 0.90 |

**미보정 상태의 정상 회전(0.26)과 진짜 슬립(0.30)이 거의 붙어 있다.**
이 상태로는 임계값을 어디에 두어도 오검출과 미검출 중 하나를 고르게 된다.

### 측정 방법

브릿지를 띄운 상태에서 도구를 실행하고, 로봇을 제자리 회전시킨다.
**도구는 모터를 구동하지 않는다.** 캘리브레이션 스크립트가 로봇을 움직이면
작업대에서 떨어뜨리기 좋으므로, 조작은 사람이 한다.

```bash
ros2 run scout2map_bridge skid_calib
# 다른 터미널에서 teleop 등으로 20~30초간 양방향 제자리 회전
```

```
  rotating  12.4s   enc    512.3 deg   imu    379.5 deg   ratio 1.350
```

값이 안정되면 Ctrl+C를 눌러 요약을 본다. 출력된 값을
`config/drive_bridge.yaml`의 `skid_factor`에 넣는다.

**실제로 주행할 바닥에서 측정한다.** 카펫과 콘크리트가 다른 값을 준다.
타이어나 적재 하중을 바꾼 경우에도 재측정한다.

### 자이로 바이어스도 함께

BNO055는 캘리브레이션 전에 정지 상태에서도 1~2 deg/s를 출력한다.
보정하지 않으면 **직진 중에도 불일치가 만들어진다.** 위 예시와 같은
1.5 deg/s 바이어스는 직진 주행에서 `slip_ratio` 0.26을 낳는다.

```bash
./tools/s2m_imu_view.py --bias     # 펌웨어 저장소
```

측정값을 rad/s로 변환해 `gyro_bias_radps`에 넣는다.
두 값이 모두 기본값이면 브릿지가 기동 시 경고한다.

### 게이트

`slip_signal_valid`가 false면 비교 자체가 성립하지 않는다는 뜻이다.
그럴듯해 보이는 무의미한 숫자는 값이 없는 것보다 나쁘다.

| 조건 | 이유 |
|---|---|
| `imu_calibrated == false` | 절대 방위와 자이로 영점을 믿을 수 없다 |
| `openloop == true` | 엔코더가 죽어 비교 대상이 없다 |
| 휠 속도 < `slip_min_wheel_speed_mps` | 정지 상태에서는 양쪽 모두 노이즈다 |

### 이 신호가 잡지 못하는 것

**직진 헛돎은 검출되지 않는다.** 양쪽 바퀴가 함께 헛돌면 엔코더는 전진을
보고하지만 요레이트 변화는 없고, IMU 역시 회전이 없다고 보고한다.
**둘이 일치하므로 아무것도 걸리지 않는다.**

원리적으로는 엔코더 가속도와 IMU 전후 가속도를 비교하면 잡을 수 있으나,
`accel_x`에는 중력이 포함되어 있어 차체가 기울면 중력 성분이 새어든다.
잔해 위처럼 슬립이 실제로 일어나는 지형이 곧 기울어진 지형이므로,
보정 없이는 가장 필요한 상황에서 가장 부정확하다.

쿼터니언으로 중력을 제거하면 가능하나 IMU 캘리브레이션 품질에 의존하며,
아직 구현하지 않았다. 현재 신호는 **회전 슬립 전용**으로 다룬다.
