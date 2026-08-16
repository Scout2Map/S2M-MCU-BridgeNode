# scout2map_msgs

Scout2Map에서 노드끼리 주고받는 **데이터 형식 정의서** 패키지다.
실행되는 노드는 하나도 없고, `.msg` 정의와 빌드 설정만 들어 있다.

브릿지가 무엇을 발행하는지 알아야 하는 모든 파트(이벤트 엔진, 맵 마커 노드, 관제 서버)는
이 문서를 계약서로 삼는다.

---

## 1. `.msg`가 무엇이고 왜 빌드가 필요한가

ROS2에서 토픽으로 흐르는 데이터는 미리 정해진 타입을 가진다.
`/sensors/air_quality`에 흘러다니는 것이 아무 데이터가 아니라
"eCO2, TVOC, AQI, validity 필드를 가진 구조체"라고 송신자와 수신자가 합의해야 통신이 성립한다.
그 합의문이 `.msg` 파일이다.

`.msg`는 그냥 텍스트지만, 브릿지는 파이썬이고 이벤트 엔진은 C++일 수 있다.
그래서 `colcon build`를 돌리면 `rosidl`이 정의 하나에서 파이썬 클래스, C++ 헤더,
직렬화 코드를 함께 생성한다. `from scout2map_msgs.msg import EnvSnapshot`이 동작하는 이유다.

**따라서 `.msg`를 수정하면 반드시 재빌드하고 다시 source해야 한다.**
빌드는 패키지 디렉토리가 아니라 **워크스페이스 루트**에서 실행한다.

```bash
cd ~/scout2map_ws          # colcon은 항상 워크스페이스 루트에서 돌린다
colcon build --packages-select scout2map_msgs
source install/setup.bash  # 새 터미널마다 필요하다
```

수정한 정의가 실제로 반영됐는지는 아래로 확인한다.
방금 추가한 필드가 목록에 보이면 성공이다.

```bash
ros2 interface show scout2map_msgs/msg/EnvSnapshot
```

이 단계를 건너뛰면 노드가 예전 정의를 그대로 쓰기 때문에,
"그런 필드가 없다"거나 타입이 맞지 않는다는 형태의 오류가 난다.
원인을 찾기 어려운 부류이므로 습관을 들이는 편이 낫다.

`Temperature`, `Illuminance` 같은 흔한 타입은 ROS가 `sensor_msgs`로 기본 제공하므로
정의하지 않았다. 표준에 없는 것만 여기에 만든다.

---

## 2. 모든 메시지 공통 - `header`

| 필드 | 타입 | 값 |
|---|---|---|
| `header.stamp` | `builtin_interfaces/Time` | 브릿지가 해당 라인을 **수신한** ROS 시각 |
| `header.frame_id` | `string` | 기본 `"sensor_fusion"`, 파라미터로 변경 가능 |

`stamp`은 MCU가 센서를 읽은 시각이 아니라 RPi5가 라인을 받은 시각이다.
USB CDC 전송 지연과 브릿지 큐 지연을 합쳐 대략 10~30ms 뒤처진다.
차체 속도가 약 0.228m/s이므로 위치 오차로 환산하면 수 mm 수준이라 무시해도 된다.
정밀한 시각 정렬이 필요해지면 MCU 쪽에서 타임스탬프를 실어 보내는 방식으로 바꿔야 한다.

`frame_id`의 기본값 `sensor_fusion`은 센서 퓨전 MCU에 물린 센서 묶음 전체를
하나의 좌표계로 뭉뚱그린 이름이다. 실제로는 조도 센서와 먼지 센서가 차체 위
서로 다른 위치에 붙지만, 환경값은 방향성이 없어서 위치 차이가 판정에 영향을 주지 않는다.
LiDAR나 IMU처럼 위치·자세가 중요한 센서는 각자의 프레임을 따로 가져야 하므로,
URDF를 작성할 때 `base_link` 아래에 이 프레임을 고정 변환으로 달아 둔다.

---

## 3. `EnvSnapshot` - 전 센서 최신값 통합 스냅샷

**이벤트 엔진이 쓸 메인 타입이다.** 토픽은 `/sensors/env_snapshot`, 5Hz로 발행된다.

센서마다 발행 주기가 다르므로(BH1750 5Hz, 나머지 1Hz) 개별 토픽을 직접 조인하면
서로 다른 시점의 값을 한 세트로 착각하게 된다. 이 메시지는 각 센서의 최신값에
**값의 나이(`*_age_s`)와 유효 플래그(`*_valid`)**를 붙여 그 문제를 없앤다.

| 필드 | 타입 | 단위 | 설명 |
|---|---|---|---|
| `temperature_c` | `float32` | ℃ | AHT21 최신 온도 |
| `humidity_pct` | `float32` | **%** | AHT21 최신 습도. 0~100. 비율 아님 |
| `ambient_valid` | `bool` | - | 온습도 값이 존재하고 3초 이내에 갱신됐는가 |
| `ambient_age_s` | `float32` | 초 | 온습도 값의 나이 |
| `illuminance_lux` | `float32` | lux | BH1750 최신 조도 |
| `illuminance_valid` | `bool` | - | 1초 이내 갱신 여부 |
| `illuminance_age_s` | `float32` | 초 | 조도 값의 나이 |
| `eco2_ppm` | `uint16` | ppm | ENS160 최신 등가 CO2 |
| `tvoc_ppb` | `uint16` | ppb | ENS160 최신 TVOC |
| `aqi` | `uint8` | - | ENS160 최신 AQI (1~5) |
| `ens160_validity` | `uint8` | - | ENS160 원본 validity (0~3). 4절 참조 |
| `air_quality_valid` | `bool` | - | **3초 이내 갱신 AND validity == 0** |
| `air_quality_age_s` | `float32` | 초 | 가스 값의 나이 |
| `pm1_0_ug_m3` | `uint16` | µg/m³ | PMS7003 최신값 |
| `pm2_5_ug_m3` | `uint16` | µg/m³ | PMS7003 최신값 |
| `pm10_ug_m3` | `uint16` | µg/m³ | PMS7003 최신값 |
| `particulate_valid` | `bool` | - | 5초 이내 갱신 여부 |
| `particulate_age_s` | `float32` | 초 | 먼지 값의 나이 |
| `link_ok` | `bool` | - | 시리얼 포트가 열려 있고 3초 이내에 라인이 도착했는가 |
| `mcu_uptime_ms` | `uint32` | ms | MCU 하트비트가 보고한 부팅 후 경과 시간 |

`*_age_s`의 기준 시각은 스냅샷 발행 시점이다. 스냅샷이 5Hz(200ms)로 나가므로
1Hz 센서의 `age_s`는 0.0과 1.0 사이를 톱니처럼 오르내리는 것이 정상이다.
유효 수명(3초 등)은 브릿지 파라미터로 조정할 수 있다.

### 반드시 지킬 것 - 숫자 필드는 `*_valid` 확인 후에 읽는다

센서가 죽어도 마지막 값은 캐시에 남는다. 그래서 플래그를 건너뛰면
**센서가 멎은 뒤에도 마지막 값으로 계속 이벤트가 발생**하거나,
반대로 초기값 0이 안전한 상태로 오인된다.

| 상황 | `*_valid` | `*_age_s` | 숫자 필드 |
|---|---|---|---|
| 부팅 후 한 번도 수신 안 됨 | `false` | `0.0` | `0` (메시지 기본값) |
| 수신했지만 갱신이 끊김 | `false` | 실제 나이 (예: `12.4`) | 마지막으로 받은 값 |
| 정상 | `true` | 주기 이내 | 최신 값 |

`age_s`가 0.0인데 `valid`가 false면 "데이터 없음"이고,
`age_s`가 큰데 false면 "센서가 죽었다"는 뜻이다. 어느 쪽이든 숫자를 믿으면 안 된다.

```python
# 올바른 사용
if msg.air_quality_valid and msg.tvoc_ppb > 2000:
    trigger_gas_event(msg.tvoc_ppb)

# 잘못된 사용 - 센서가 죽은 뒤에도 계속 발화한다
if msg.tvoc_ppb > 2000:
    trigger_gas_event(msg.tvoc_ppb)
```

---

## 4. `AirQuality` - ENS160 가스/공기질

토픽 `/sensors/air_quality`, 1Hz.

| 필드 | 타입 | 단위 | 값 범위 | 설명 |
|---|---|---|---|---|
| `eco2_ppm` | `uint16` | ppm | 400 ~ 65000 | **등가** CO2. 실측이 아니다. 아래 설명 참조 |
| `tvoc_ppb` | `uint16` | ppb | 0 ~ 65000 | 총 휘발성 유기화합물 농도 |
| `aqi` | `uint8` | - | 1 ~ 5 | UBA 공기질 지수. 1이 가장 좋고 5가 가장 나쁘다 |
| `validity` | `uint8` | - | 0 ~ 3 | 아래 상수 표 참조 |
| `operational` | `bool` | - | - | `validity == 0`일 때만 true. 브릿지가 계산해 넣는 편의 필드 |

### `validity` 상수

메시지 정의에 상수로 들어 있으므로 숫자를 직접 쓰지 말고 상수를 참조한다.

| 상수 | 값 | 의미 | 값을 써도 되는가 |
|---|---|---|---|
| `VALIDITY_NORMAL` | 0 | 정상 동작 | **그렇다** |
| `VALIDITY_WARMUP` | 1 | 워밍업 중 (전원 인가 후 약 3분) | 아니다 |
| `VALIDITY_INITIAL_START` | 2 | 초기 구동 단계 (새 부품 첫 사용 후 약 1시간) | 아니다 |
| `VALIDITY_INVALID` | 3 | 출력 무효 | 아니다 |

브릿지는 워밍업 중인 값도 필터링하지 않고 그대로 올린다.
필터링 여부를 소비자가 결정할 수 있어야 하기 때문이다.
가스 임계값 판정은 반드시 `operational == true`일 때만 수행한다.

### `eco2_ppm`은 이산화탄소 농도가 아니다

ENS160은 CO2 센서가 아니라 MOX 방식 가스 센서다.
검출한 VOC 농도로부터 실내 CO2 농도를 역산해 내놓는 **지표**이지 실측값이 아니다.
사람 호흡으로 인한 CO2 상승에는 비교적 잘 따라오지만,
화재 현장의 실제 CO2 농도를 측정하는 용도로는 신뢰할 수 없다.

**"가스 고농도" 이벤트는 `tvoc_ppb`를 주 지표로, `eco2_ppm`을 보조로 쓰는 편이 안전하다.**

---

## 5. `Particulate` - PMS7003 미세먼지

토픽 `/sensors/particulate`, 최대 1Hz(이벤트 구동).

| 필드 | 타입 | 단위 | 설명 |
|---|---|---|---|
| `pm1_0_ug_m3` | `uint16` | µg/m³ | 1.0µm 이하 입자 농도 |
| `pm2_5_ug_m3` | `uint16` | µg/m³ | 2.5µm 이하 입자 농도 |
| `pm10_ug_m3` | `uint16` | µg/m³ | 10µm 이하 입자 농도 |

PMS7003은 한 프레임에 CF=1 표준입자 기준값과 대기환경 기준값 **두 세트**를 함께 내보낸다.
브릿지는 MCU가 보내준 값을 변환 없이 전달하므로, 어느 쪽이 올라오는지는
펌웨어 `pms7003.c` 파서가 프레임의 어느 오프셋을 읽는지에 달려 있다.
실내 환경 판정에는 대기환경 기준값을 쓰는 것이 일반적이다.

데이터시트상 정확도가 보장되는 구간은 대략 0~500 µg/m³이고 그 위는 참고값이다.
화재 현장처럼 농도가 포화되는 상황에서는 절대값보다 **상승 추세**로 판단하는 편이 낫다.

---

## 6. `BridgeStatus` - 링크 및 노드 상태

토픽 `/bridge/status`, 1Hz.
QoS가 TRANSIENT_LOCAL이므로 나중에 뜬 노드도 마지막 상태를 즉시 받는다.

| 필드 | 타입 | 설명 |
|---|---|---|
| `port` | `string` | 실제로 열려고 시도 중인 장치 경로 |
| `port_open` | `bool` | 시리얼 포트 개방 상태 |
| `link_ok` | `bool` | 포트가 열려 있고 타임아웃 이내에 라인이 도착했는가 |
| `last_line_age_s` | `float32` | 마지막 라인 수신 후 경과 시간. **아직 한 줄도 못 받았으면 `-1.0`** |
| `mcu_uptime_ms` | `uint32` | MCU 부팅 후 경과 ms. 32비트라 약 49.7일에서 자연 롤오버한다 |
| `mcu_reboot_count` | `uint32` | `uptime_ms`가 뒤로 점프한 횟수. 전원 계통 문제 추적용 |
| `lines_received` | `uint64` | 수신한 라인 총 개수 |
| `parse_errors` | `uint64` | JSON 파싱 실패 또는 필수 필드 누락으로 버린 라인 수 |
| `unknown_src` | `uint64` | JSON은 유효하나 `src` 값을 모르는 라인 수 |
| `aht21_present` | `bool` | MCU 부팅 라인이 보고한 초기화 성공 여부 |
| `ens160_present` | `bool` | 동일 |
| `bh1750_present` | `bool` | 동일 |
| `pms7003_seen` | `bool` | PMS7003은 초기화 핸드셰이크가 없으므로 유효 프레임 수신 여부로 판정한다 |

`unknown_src`는 펌웨어에 센서를 추가하고 브릿지 라우팅을 고치지 않았을 때 올라간다.

`*_present`는 MCU가 부팅 라인을 보낸 뒤에야 채워진다.
브릿지를 MCU보다 늦게 띄우면 부팅 라인을 놓칠 수 있고, 이때 세 값 모두 false로 남는다.
**`lines_received`가 늘고 있는데 `*_present`가 false라면 초기화 실패가 아니라
부팅 라인을 놓친 것**이므로, MCU를 리셋해 다시 확인한다.

---

## 7. 표준 메시지를 쓰는 토픽

온도·습도·조도는 ROS 표준 타입을 그대로 쓴다.
`rqt_plot`, rviz, `ros2 topic echo`가 커스텀 타입 없이 바로 붙기 때문이다.

| 토픽 | 타입 | 주요 필드 | 단위 | 값 범위 |
|---|---|---|---|---|
| `/sensors/temperature` | `sensor_msgs/Temperature` | `temperature` | ℃ | -40 ~ 120 (AHT21 정확도 ±0.5℃) |
| `/sensors/humidity` | `sensor_msgs/RelativeHumidity` | `relative_humidity` | **비율** | 0.0 ~ 1.0 |
| `/sensors/illuminance` | `sensor_msgs/Illuminance` | `illuminance` | lux | 1 ~ 65535 (분해능 1 lux) |

### 함정 - 습도 단위가 두 가지다

`sensor_msgs/RelativeHumidity`는 표준상 0.0~1.0 비율이라 브릿지가 100으로 나눠 발행한다.
습도 40%면 `0.4`가 들어온다.
반면 `EnvSnapshot.humidity_pct`는 0~100 퍼센트 값이다. 섞어 쓰지 않도록 주의한다.

### `variance` 필드는 무엇인가

세 표준 메시지 모두 `variance` 필드를 갖는다. 그 측정값이 얼마나 흔들리는지를
통계적 분산(표준편차의 제곱)으로 표현한 값으로, 센서를 여러 개 융합하거나
칼만 필터를 쓸 때 "이 센서를 얼마나 믿을지" 가중치로 쓰인다.
분산이 작을수록 큰 가중치가 걸린다.

ROS 표준은 **0.0을 "분산을 모른다"는 뜻으로 약속**해 두었다. 오차가 0이라는 뜻이 아니다.
브릿지는 실측으로 분산을 구한 적이 없으므로 전부 0.0으로 채운다.
현재 구조에서는 임계값 비교만 하므로 이 필드를 쓰는 곳이 없다.

나중에 온도나 조도를 다른 추정값과 융합할 일이 생기면 데이터시트 정확도로 근사할 수 있다.
AHT21의 온도 정확도 ±0.5℃를 표준편차 약 0.25℃로 보면 `variance = 0.0625`가 된다.
다만 이는 어림값이고, 정확한 값은 정지 상태에서 수백 샘플을 모아 실측해야 얻어진다.

---

## 8. 메시지를 추가·수정할 때

1. `msg/` 아래에 `.msg` 파일을 만들거나 고친다.
2. 새 파일이면 `CMakeLists.txt`의 `rosidl_generate_interfaces` 목록에 등록한다.
3. 재빌드하고 다시 source한다.
4. 이 문서에 필드 설명을 추가한다.

**필드를 삭제하거나 타입을 바꾸는 것은 호환성을 깨는 변경**이다.
이미 그 필드를 구독하는 노드가 있다면 함께 수정해야 하므로, 팀에 먼저 공유한다.
필드 추가는 비교적 안전하다.

주행 제어 MCU(STM32) 관련 메시지도 이 패키지에 함께 추가할 예정이며,
토픽은 `drive/` 네임스페이스를 쓴다.

---

## 9. 주행 제어 메시지 (STM32)

주행 제어 MCU에서 오는 값은 대부분 ROS 표준 타입으로 나간다.
Nav2와 SLAM이 그대로 받아먹는 타입을 커스텀으로 바꾸면 손해이기 때문이다.

| 토픽 | 타입 | 주기 |
|---|---|---|
| `/drive/odom` | `nav_msgs/Odometry` | 50Hz |
| `/drive/imu` | `sensor_msgs/Imu` | 50Hz |
| `/drive/range` | `sensor_msgs/Range` | 50Hz |
| `/drive/battery` | `sensor_msgs/BatteryState` | 50Hz |
| `/drive/status` | `scout2map_msgs/DriveStatus` | 10Hz |
| `/drive/diagnostics` | `scout2map_msgs/DriveDiagnostics` | 요청 시 |

커스텀이 필요한 것은 표준에 대응물이 없는 두 가지뿐이다.
모터 듀티, 폴트 래치 상태, 엔코더 원시 카운트, BNO055 캘리브레이션 진행도는
어느 표준 메시지에도 자리가 없다.

### `DriveStatus`

주행부 상태 전반이다. 상태 비트필드를 원본 그대로(`status_flags`) 실으면서
동시에 개별 `bool`로도 풀어 두었으므로, 소비자가 마스킹할 필요가 없다.

특히 중요한 두 필드가 있다.

**`openloop`** — 엔코더 신호가 끊겨 MCU가 개루프로 폴백한 상태다.
이때 오도메트리는 명령값에 기반한 추정이므로 신뢰할 수 없다.
브릿지가 `Odometry` 공분산을 100배로 부풀리지만, 이벤트 판정 쪽에서도
이 플래그를 봐야 한다.

**`estop_latched`** — 래치되어 있으므로 속도 명령을 아무리 보내도 움직이지
않는다. `/drive/clear_fault` 서비스를 호출해야 풀린다.
"명령을 보냈는데 로봇이 안 움직인다"의 첫 번째 확인 대상이다.

`calib_sys` / `calib_gyro` / `calib_accel` / `calib_mag`는 각각 0~3이며
3이 완료다. 지자계가 3에 도달하기 전에는 절대 방위가 드리프트한다.

### `DriveDiagnostics`

요청했을 때만 발행된다. 원시 ADC 카운트와 I2C 오류 카운터가 들어 있어,
없는 센서와 멈춘 버스와 잘못된 주소를 구분할 수 있다.
스케일된 값만으로는 이 셋이 모두 그럴듯해 보인다.

```bash
ros2 service call /drive/request_diagnostics std_srvs/srv/Trigger
ros2 topic echo /drive/diagnostics --once
```

### 표준 메시지 쪽에서 주의할 점

**`Imu`의 각속도는 Z만 실린다.** 와이어에 `gyro_z`밖에 없기 때문이다.
롤/피치 각속도 공분산은 1e6으로 채워 "값 없음"을 표시한다.

**`Range`의 두 특수값이 다른 뜻이다.** 측정 범위 밖은 `+inf`,
최소 거리 이내(장애물이 코앞에 있으나 거리 불명)는 `min_range`로 나간다.
후자를 `inf`로 처리하면 바로 앞 장애물이 빈 공간으로 기록되므로 반대로 했다.

**`BatteryState.voltage`가 `NaN`일 수 있다.** ADC가 아직 값을 보고하지
않았다는 뜻이며, 이때 `present`가 false다. 0V로 오해하면 안 된다.
`percentage`는 항상 `NaN`이다. 이 팩의 방전 곡선을 특성화한 적이 없고,
모터 부하 아래에서 추정하면 모르는 것만 못하다.

와이어 포맷 자체는 [`../scout2map_bridge/PROTOCOL.md`](../scout2map_bridge/PROTOCOL.md)에 있다.
