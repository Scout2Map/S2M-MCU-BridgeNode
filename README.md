# scout2map_bridge

Raspberry Pi Pico 2 센서 퓨전 MCU와 ROS2 사이를 연결하는 브릿지 노드다.
MCU가 USB CDC로 내보내는 JSON 라인을 파싱해 ROS2 토픽으로 재발행한다.

이 패키지의 목적은 **다른 파트가 시리얼이나 JSON을 전혀 몰라도 되게 만드는 것**이다.
이벤트 엔진, SLAM 파이프라인, 관제 서버는 아래 토픽만 구독하면 된다.

---

## 1. 토픽 계약 (Topic Contract)

| 토픽 | 타입 | 주기 | 용도 |
|---|---|---|---|
| `/sensors/temperature` | `sensor_msgs/Temperature` | 1Hz | 온도. rqt_plot, rviz에서 바로 확인 가능 |
| `/sensors/humidity` | `sensor_msgs/RelativeHumidity` | 1Hz | 습도. **0.0~1.0 비율**이다 (표준 규약) |
| `/sensors/illuminance` | `sensor_msgs/Illuminance` | 5Hz | 조도 |
| `/sensors/air_quality` | `scout2map_msgs/AirQuality` | 1Hz | eCO2, TVOC, AQI, ENS160 validity |
| `/sensors/particulate` | `scout2map_msgs/Particulate` | ~1Hz | PM1.0 / PM2.5 / PM10 |
| `/sensors/env_snapshot` | `scout2map_msgs/EnvSnapshot` | 5Hz | **전 센서 최신값 통합 스냅샷** |
| `/bridge/status` | `scout2map_msgs/BridgeStatus` | 1Hz | 링크 상태, 파싱 에러, MCU 리부트 카운트 |
| `/sensors/raw_json` | `std_msgs/String` | 가변 | 원본 라인 미러링. `publish_raw_json:=true`일 때만 |

QoS는 전부 RELIABLE / KEEP_LAST(depth 10)이다.
데이터 레이트가 초당 수 건 수준이라 신뢰성 전송 비용이 사실상 없고,
이벤트 엔진이 임계값 돌파 순간의 샘플을 조용히 놓치는 상황을 막는 편이 훨씬 중요하다.
단 `/bridge/status`만 TRANSIENT_LOCAL이라, 나중에 뜬 노드도 마지막 상태를 즉시 받는다.

---

## 2. 메시지 필드 상세

### 2.1 모든 메시지 공통 - `header`

| 필드 | 타입 | 값 |
|---|---|---|
| `header.stamp` | `builtin_interfaces/Time` | 브릿지가 해당 라인을 **수신한** ROS 시각 |
| `header.frame_id` | `string` | 기본 `"sensor_fusion"`, 파라미터로 변경 가능 |

`stamp`은 MCU가 센서를 읽은 시각이 아니라 RPi5가 라인을 받은 시각이다.
USB CDC 전송 지연 + 브릿지 큐 지연을 합쳐 대략 10~30ms 뒤처진다.
차체 속도가 약 0.228m/s이므로 위치 오차로 환산하면 수 mm 수준이라 무시해도 된다.
정밀한 시각 정렬이 필요해지면 MCU 쪽에서 타임스탬프를 실어 보내는 방식으로 바꿔야 한다.

`frame_id`의 기본값 `sensor_fusion`은 센서 퓨전 MCU에 물린 센서 묶음 전체를 하나의
좌표계로 뭉뚱그린 이름이다. 실제로는 조도 센서와 먼지 센서가 차체 위 서로 다른 위치에
붙지만, 환경값은 방향성이 없어서 위치 차이가 판정에 영향을 주지 않는다.
LiDAR나 IMU처럼 위치·자세가 중요한 센서는 각자의 프레임을 따로 가져야 하므로,
나중에 URDF를 작성할 때 `base_link` 아래에 이 프레임을 고정 변환으로 달아 두면 된다.

### 2.2 `sensor_msgs/Temperature` - `/sensors/temperature`

| 필드 | 타입 | 단위 | 값 범위 | 설명 |
|---|---|---|---|---|
| `temperature` | `float64` | ℃ | -40 ~ 120 | AHT21 원본값. 정확도 ±0.5℃ |
| `variance` | `float64` | (℃)² | 0.0 고정 | 측정 오차의 분산. 아래 설명 참조 |

**`variance` 필드가 무엇인가:**
`sensor_msgs` 계열 메시지가 공통으로 갖는 필드로, 그 측정값이 얼마나 흔들리는지를
통계적 분산(표준편차의 제곱)으로 표현한 값이다. 센서를 여러 개 융합하거나 칼만 필터를
쓸 때, "이 센서 값을 얼마나 믿을지"를 결정하는 가중치로 쓰인다.
분산이 작을수록 그 값에 더 큰 가중치가 걸린다.

ROS 표준은 **0.0을 "분산을 모른다"는 뜻으로 약속**해 두었다. 오차가 0이라는 뜻이 아니다.
이 브릿지는 실측으로 분산을 구한 적이 없으므로 전부 0.0으로 채운다.
현재 구조에서는 임계값 비교만 하기 때문에 이 필드를 쓰는 곳이 없다.

나중에 온도나 조도를 다른 추정값과 융합할 일이 생기면, 데이터시트 정확도로부터
대략적인 값을 넣어 줄 수 있다. 예를 들어 AHT21의 온도 정확도가 ±0.5℃이고 이를
표준편차 약 0.25℃로 보면 `variance = 0.0625`가 된다.
다만 이는 어디까지나 근사치이고, 정확한 값은 정지 상태에서 수백 샘플을 모아
실측 분산을 계산해야 얻어진다.

### 2.3 `sensor_msgs/RelativeHumidity` - `/sensors/humidity`

| 필드 | 타입 | 단위 | 값 범위 | 설명 |
|---|---|---|---|---|
| `relative_humidity` | `float64` | **비율** | 0.0 ~ 1.0 | AHT21 값(%)을 100으로 나눈 값. 40%면 `0.4`가 들어온다 |
| `variance` | `float64` | - | 0.0 고정 | 측정 오차의 분산. 2.2절 아래 설명 참조 |

이 토픽만 비율이고 `EnvSnapshot.humidity_pct`는 퍼센트다. 반드시 구분한다.

### 2.4 `sensor_msgs/Illuminance` - `/sensors/illuminance`

| 필드 | 타입 | 단위 | 값 범위 | 설명 |
|---|---|---|---|---|
| `illuminance` | `float64` | lux | 1 ~ 65535 | BH1750 원본값. 분해능 1 lux |
| `variance` | `float64` | (lux)² | 0.0 고정 | 측정 오차의 분산. 2.2절 아래 설명 참조 |

BH1750은 사람 눈의 시감 특성에 가까운 응답을 가진다. 저조도 이벤트 임계값은
이 값을 그대로 쓰면 되고, 별도 감마 보정이 필요하지 않다.

### 2.5 `scout2map_msgs/AirQuality` - `/sensors/air_quality`

| 필드 | 타입 | 단위 | 값 범위 | 설명 |
|---|---|---|---|---|
| `eco2_ppm` | `uint16` | ppm | 400 ~ 65000 | **등가** CO2. NDIR 실측이 아니라 VOC 측정값에서 ENS160 내부 알고리즘이 추정한 값이다 |
| `tvoc_ppb` | `uint16` | ppb | 0 ~ 65000 | 총 휘발성 유기화합물 농도 |
| `aqi` | `uint8` | - | 1 ~ 5 | UBA 공기질 지수. 1이 가장 좋고 5가 가장 나쁘다 |
| `validity` | `uint8` | - | 0 ~ 3 | 아래 상수 참조 |
| `operational` | `bool` | - | - | `validity == 0`일 때만 true. 브릿지가 계산해서 넣는 편의 필드 |

`validity` 상수 (메시지 정의에 함께 들어 있다):

| 상수 | 값 | 의미 | 값을 써도 되는가 |
|---|---|---|---|
| `VALIDITY_NORMAL` | 0 | 정상 동작 | **그렇다** |
| `VALIDITY_WARMUP` | 1 | 워밍업 중 (전원 인가 후 약 3분) | 아니다 |
| `VALIDITY_INITIAL_START` | 2 | 초기 구동 단계 (새 부품 첫 사용 후 약 1시간) | 아니다 |
| `VALIDITY_INVALID` | 3 | 출력 무효 | 아니다 |

`eco2_ppm`은 "이산화탄소 농도"가 아니다. ENS160은 CO2 센서가 아니라 MOX 가스 센서이고,
검출한 VOC 농도로부터 실내 CO2 농도를 역산해 내놓는 지표다. 사람 호흡으로 인한 CO2 상승에는
비교적 잘 따라오지만, 화재 현장의 실제 CO2 농도를 측정하는 용도로는 신뢰할 수 없다.
보고서상 "가스 고농도" 이벤트는 `tvoc_ppb`를 주 지표로, `eco2_ppm`을 보조로 쓰는 편이 안전하다.

### 2.6 `scout2map_msgs/Particulate` - `/sensors/particulate`

| 필드 | 타입 | 단위 | 설명 |
|---|---|---|---|
| `pm1_0_ug_m3` | `uint16` | µg/m³ | 1.0µm 이하 입자 농도 |
| `pm2_5_ug_m3` | `uint16` | µg/m³ | 2.5µm 이하 입자 농도 |
| `pm10_ug_m3` | `uint16` | µg/m³ | 10µm 이하 입자 농도 |

PMS7003은 한 프레임 안에 CF=1 표준입자 기준값과 대기환경 기준값 **두 세트**를 함께 내보낸다.
브릿지는 MCU가 보내준 값을 그대로 전달할 뿐 변환하지 않으므로,
어느 쪽이 올라오는지는 `pms7003.c`의 파서가 프레임의 어느 오프셋을 읽는지에 달려 있다.
실내 환경 판정에는 대기환경 기준값을 쓰는 것이 일반적이다.

데이터시트상 정확도가 보장되는 구간은 대략 0~500 µg/m³이고 그 위는 참고값이다.
화재 현장처럼 농도가 포화되는 상황에서는 절대값보다 **상승 추세**로 판단하는 편이 낫다.

### 2.7 `scout2map_msgs/EnvSnapshot` - `/sensors/env_snapshot`

이벤트 엔진이 쓸 메인 토픽이다. 센서별 최신값 + 나이 + 유효 플래그로 구성된다.

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
| `ens160_validity` | `uint8` | - | ENS160 원본 validity (0~3) |
| `air_quality_valid` | `bool` | - | **3초 이내 갱신 AND validity == 0** |
| `air_quality_age_s` | `float32` | 초 | 가스 값의 나이 |
| `pm1_0_ug_m3` | `uint16` | µg/m³ | PMS7003 최신값 |
| `pm2_5_ug_m3` | `uint16` | µg/m³ | PMS7003 최신값 |
| `pm10_ug_m3` | `uint16` | µg/m³ | PMS7003 최신값 |
| `particulate_valid` | `bool` | - | 5초 이내 갱신 여부 |
| `particulate_age_s` | `float32` | 초 | 먼지 값의 나이 |
| `link_ok` | `bool` | - | 시리얼 포트가 열려 있고 3초 이내에 라인이 도착했는가 |
| `mcu_uptime_ms` | `uint32` | ms | MCU 하트비트가 보고한 부팅 후 경과 시간 |

`*_age_s`의 기준 시각은 스냅샷 발행 시점이다. 스냅샷은 5Hz(200ms)로 나가므로
1Hz 센서의 `age_s`는 0.0에서 1.0 사이를 톱니처럼 오르내리는 것이 정상이다.

**함정 - 값이 없을 때와 값이 오래됐을 때의 구분:**

| 상황 | `*_valid` | `*_age_s` | 숫자 필드 |
|---|---|---|---|
| 부팅 후 한 번도 수신 안 됨 | `false` | `0.0` | `0` (메시지 기본값) |
| 수신했지만 갱신이 끊김 | `false` | 실제 나이 (예: `12.4`) | 마지막으로 받은 값 |
| 정상 | `true` | 주기 이내 | 최신 값 |

`age_s`가 0.0인데 `valid`가 false면 "데이터 없음"이고, `age_s`가 큰데 false면 "센서가 죽었다"는
뜻이다. 어느 쪽이든 **숫자 필드를 읽으면 안 된다.** `valid` 확인 없이 `eco2_ppm`을 그대로
임계값과 비교하면, 센서가 죽은 뒤에도 마지막 값으로 계속 이벤트가 발생하거나 반대로
0이 안전한 값으로 오인된다.

### 2.8 `scout2map_msgs/BridgeStatus` - `/bridge/status`

| 필드 | 타입 | 설명 |
|---|---|---|
| `port` | `string` | 실제로 열려고 시도 중인 장치 경로 |
| `port_open` | `bool` | 시리얼 포트 개방 상태 |
| `link_ok` | `bool` | 포트가 열려 있고 `link_timeout_s` 이내에 라인이 도착했는가 |
| `last_line_age_s` | `float32` | 마지막 라인 수신 후 경과 시간. **아직 한 줄도 못 받았으면 `-1.0`** |
| `mcu_uptime_ms` | `uint32` | MCU 부팅 후 경과 ms. 32비트라 약 49.7일에서 자연 롤오버한다 |
| `mcu_reboot_count` | `uint32` | `uptime_ms`가 뒤로 점프한 횟수. 전원 계통 문제 추적용 |
| `lines_received` | `uint64` | 수신한 라인 총 개수 |
| `parse_errors` | `uint64` | JSON 파싱 실패 또는 필수 필드 누락으로 버린 라인 수 |
| `unknown_src` | `uint64` | JSON은 유효하나 `src` 값을 모르는 라인 수. 펌웨어에 센서를 추가하고 브릿지를 안 고치면 여기가 올라간다 |
| `aht21_present` | `bool` | MCU 부팅 라인이 보고한 초기화 성공 여부 |
| `ens160_present` | `bool` | 동일 |
| `bh1750_present` | `bool` | 동일 |
| `pms7003_seen` | `bool` | PMS7003은 초기화 핸드셰이크가 없으므로, 유효 프레임을 한 번이라도 받았는지로 판정한다 |

`*_present` 세 개는 MCU가 부팅 라인을 보낸 뒤에야 채워진다.
브릿지를 MCU보다 먼저 띄우면 부팅 라인을 놓칠 수 있고, 이때는 세 값 모두 false로 남는다.
`lines_received`가 늘고 있는데 `*_present`가 false면 초기화 실패가 아니라
부팅 라인을 놓친 것이므로, MCU를 리셋해서 다시 확인한다.

---

## 3. 어느 토픽을 써야 하는가

**이벤트 엔진(임계값 판별)** → `/sensors/env_snapshot` 하나만 구독하면 된다.
센서마다 주기가 다르기 때문에 개별 토픽을 직접 조인하면 "온도는 방금 값인데
가스는 3초 전 값"인 상태로 판단하게 된다. 스냅샷은 이 문제를 이미 해결한 형태다.

**맵 마커 생성** → `/sensors/env_snapshot`을 구독하고, odom/TF로 이동 거리를 계산해
10cm마다 스냅샷을 좌표에 찍는다. 브릿지는 좌표를 모르므로 이 로직은 별도 노드가 담당한다.

**단순 모니터링·그래프** → `/sensors/temperature` 등 표준 메시지 토픽.
`ros2 topic echo`, `rqt_plot`, rviz 플러그인이 커스텀 메시지 없이 바로 붙는다.

**디버깅** → `publish_raw_json:=true`로 켜고 `/sensors/raw_json`을 echo한다.

---

## 4. 놓치기 쉬운 규약 세 가지

**하나. 습도 단위가 두 가지다.**
`sensor_msgs/RelativeHumidity`는 표준상 0.0~1.0 비율이라 브릿지가 100으로 나눠 발행한다.
반면 `EnvSnapshot.humidity_pct`는 0~100 % 값이다. 섞어 쓰지 않도록 주의한다.

**둘. ENS160의 `validity`는 버리지 않는다.**
브릿지는 워밍업 중인 값도 필터링하지 않고 그대로 올린다. 가스 임계값 판정은 반드시
`operational == true`(개별 토픽) 또는 `air_quality_valid == true`(스냅샷)일 때만 수행한다.

**셋. 값의 부재와 값의 정체(停滯)는 다르다.**
센서가 죽어도 마지막 값은 캐시에 남는다. 2.7절의 함정 표를 참고한다.

---

## 5. 빌드 및 실행

```bash
cd ~/scout2map_ws
sudo apt install python3-serial
colcon build --packages-select scout2map_msgs scout2map_bridge
source install/setup.bash
```

udev 규칙을 먼저 설치하면 `/dev/ttyACM*` 번호가 바뀌어도 경로가 고정된다.

```bash
lsusb | grep -i raspberry          # VID:PID 확인 (기본값 2e8a:000a)
sudo cp src/scout2map_bridge/udev/99-scout2map-pico.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout $USER     # 로그아웃 후 재로그인 필요
```

실행한다.

```bash
ros2 launch scout2map_bridge pico_bridge.launch.py
# 또는 파라미터를 직접 넘겨서
ros2 run scout2map_bridge pico_bridge --ros-args -p port:=/dev/ttyACM0 -p publish_raw_json:=true
```

확인한다.

```bash
ros2 topic echo /bridge/status --once
ros2 topic hz /sensors/env_snapshot
ros2 interface show scout2map_msgs/msg/EnvSnapshot   # 필드 목록 직접 확인
```

---

## 6. 파라미터

| 이름 | 기본값 | 설명 |
|---|---|---|
| `port` | `/dev/scout2map_pico` | 시리얼 장치 경로 |
| `baudrate` | `115200` | USB CDC라 실제로는 무시되지만 pyserial이 요구한다 |
| `frame_id` | `sensor_fusion` | 모든 메시지의 `header.frame_id` |
| `snapshot_rate_hz` | `5.0` | 스냅샷 발행 주기 |
| `status_rate_hz` | `1.0` | 상태 발행 주기 |
| `publish_raw_json` | `false` | 원본 라인 미러링 여부 |
| `stale_ambient_s` | `3.0` | AHT21 값의 유효 수명 |
| `stale_illuminance_s` | `1.0` | BH1750 값의 유효 수명 |
| `stale_air_quality_s` | `3.0` | ENS160 값의 유효 수명 |
| `stale_particulate_s` | `5.0` | PMS7003 값의 유효 수명 |
| `link_timeout_s` | `3.0` | 이 시간 동안 라인이 없으면 링크 다운으로 본다 |

`stale_*` 값은 각 센서의 발행 주기보다 넉넉히 크게 잡아야 한다.
1Hz 센서에 1.0초를 주면 지터 한 번에 `valid`가 깜빡거린다.

---

## 7. 구독 예시

Python:

```python
from scout2map_msgs.msg import EnvSnapshot

def on_snapshot(msg):
    # Never read a numeric field without checking its valid flag first
    if msg.air_quality_valid and msg.tvoc_ppb > 2000:
        trigger_gas_event(msg.tvoc_ppb)

self.create_subscription(EnvSnapshot, '/sensors/env_snapshot', on_snapshot, 10)
```

C++:

```cpp
#include "scout2map_msgs/msg/env_snapshot.hpp"

sub_ = create_subscription<scout2map_msgs::msg::EnvSnapshot>(
    "/sensors/env_snapshot", 10,
    [this](const scout2map_msgs::msg::EnvSnapshot::SharedPtr msg) {
      if (msg->illuminance_valid && msg->illuminance_lux < 50.0f) {
        // low-light event
      }
    });
```

---

## 8. 설계 노트

- **시리얼 읽기는 별도 스레드**에서 수행하고, 수신 라인은 deque에 넣은 뒤
  ROS 타이머(100Hz)가 꺼내 발행한다. 발행이 전부 단일 실행 스레드에서 일어나므로
  퍼블리셔 동시 접근 문제가 생기지 않는다. 최대 지연은 10ms 수준이라 무시할 만하다.
- **재연결은 자동이다.** USB가 빠지거나 MCU가 리셋되면 1초 간격으로 포트 재개방을 시도한다.
  노드를 다시 띄울 필요가 없다.
- **MCU 리부트를 감지한다.** 하트비트의 `uptime_ms`가 뒤로 점프하면 리부트로 간주하고
  `mcu_reboot_count`를 올린다. 이때 `pms7003_seen`도 false로 되돌린다.
- **깨진 라인은 노드를 죽이지 않는다.** JSON 파싱 실패, 필드 누락, NaN/inf 값은 모두
  카운터만 올리고 폐기한다. 로그는 50건마다 한 번씩만 남겨 콘솔 폭주를 막는다.
- **역방향 통신은 없다.** 현재 Pico2 펌웨어는 수신 명령을 처리하지 않으므로
  브릿지도 송신 경로를 두지 않았다.

---

## 9. 다음 단계

- STM32 주행 제어 MCU용 브릿지는 별도 노드로 만든다. 프로토콜이 다르고(바이너리 프레이밍 예정)
  양방향이며 명령 워치독이 필요하므로 이 노드에 합치지 않는다.
  다만 메시지는 `scout2map_msgs`에 함께 추가하고 토픽은 `drive/` 네임스페이스를 쓴다.
- 거리 기반 이벤트 마커 노드(`/sensors/env_snapshot` + TF → 마커 발행)를 추가한다.
- 펌웨어에 센서를 추가할 때는 브릿지의 `_handle_line` 라우팅에도 `src`를 등록해야 한다.
  등록을 잊으면 `BridgeStatus.unknown_src`가 올라간다.
