# S2M-MCU-BridgeNode

**Scout2Map** — 다중 센서 기반 환경 적응형 정찰 UGV의 **센서 퓨전 MCU(Raspberry Pi Pico 2) 및 주행 제어 MCU(STM32)와 ROS2를 연결하는 브릿지**다.
RPi5에서 동작하며, ROS2 패키지 두 개로 구성된다.

---

## 1. 시스템 내 위치

```
[ 환경 센서 ]                 [ 센서 퓨전 MCU ]        [ RPi5 / ROS2 ]
AHT21   ─ I2C1 ┐
ENS160  ─ I2C1 ┤
BH1750  ─ I2C0 ┼──────────▶  Raspberry Pi Pico 2  ──▶  sensor_bridge  ──▶  /sensors/*
PMS7003 ─ UART ┘              (JSON per line)                                │
                                                                             ▼
[ 구동계 ]                    [ 주행 제어 MCU ]                          이벤트 엔진
BTS7960 ×2 ────┐                                                        맵 마커 노드
엔코더 ×2  ────┼──────────▶  STM32F103C8T6    ◀──▶  drive_bridge  ──▶  관제 서버
BNO055    ────┤              (binary + CRC16)          │                 Nav2 / SLAM
GP2D120X  ────┘                    ▲                   │
                                   └───── /cmd_vel ────┘
```

주행 링크만 양방향이다. 센서 브릿지가 멈추면 데이터가 비지만,
주행 브릿지가 잘못 멈추면 차체가 마지막 명령대로 계속 굴러간다.
그래서 명령 경로에 워치독이 두 겹으로 들어간다.

**이 레포의 목적은 다른 파트가 시리얼도 프로토콜도 몰라도 되게 만드는 것**이다.
이벤트 엔진은 `/sensors/env_snapshot`을, Nav2는 `/drive/odom`과 `/cmd_vel`을
쓰면 되고, 어느 쪽도 USB 아래를 알 필요가 없다.

관련 레포는 다음과 같이 나뉜다. 경계는 "어느 하드웨어에서 실행되는가"이다.

| 레포 | 실행 위치 | 빌드 체계 |
|---|---|---|
| `S2M-FW-SensorFusion` | Pico 2 | pico-sdk |
| [`S2M-FW-DrivingControl`](https://github.com/Scout2Map/S2M-FW-DrivingControl) | STM32 | arm-none-eabi-gcc + Makefile |
| **이 레포** | **RPi5** | **colcon (ROS2)** |

두 MCU용 브릿지 노드가 모두 RPi5에서 돌기 때문에 이 레포에 함께 있다.
펌웨어와 브릿지는 실행 하드웨어가 다르므로 레포를 나눈다.

---

## 2. 구성

```
scout2map-bridge/
├── scout2map_msgs/       # 메시지 타입 정의. 실행되는 노드는 없다
│   ├── msg/              #   AirQuality, Particulate, EnvSnapshot, SensorStatus
│   └── README.md         #   ★ 필드별 상세 레퍼런스
└── scout2map_bridge/     # 브릿지 노드 본체
    ├── scout2map_bridge/
    │   ├── sensor_bridge_node.py   # 센서 퓨전 MCU (Pico 2)
    │   ├── drive_bridge_node.py  # 주행 제어 MCU (STM32)
    │   ├── drive_protocol.py     #   STM32 와이어 포맷
    │   ├── serial_link.py        #   포트 개방/재연결 공용
    │   └── fake_sensor_node.py   # 하드웨어 없이 쓰는 가짜 퍼블리셔 (6절)
    ├── PROTOCOL.md       #   ★ STM32 바이너리 프로토콜 명세
    ├── config/           #   파라미터 YAML
    ├── launch/
    ├── udev/             #   장치 경로 고정 규칙
    └── README.md         #   ★ 노드 동작 방식, 파라미터, 트러블슈팅
```

두 패키지로 나뉜 이유는 `.msg` 컴파일이 CMake 기반이라 파이썬 패키지 안에서 돌지 않기 때문이고,
의존성 측면에서도 옳다. 이벤트 엔진은 `scout2map_msgs`만 있으면 되고
브릿지 코드나 pyserial까지 끌어올 이유가 없다.

**이 레포는 워크스페이스가 아니라 패키지 묶음이다.**
따라서 클론한 자리에서 바로 빌드되지 않고, ROS2 워크스페이스의 `src/` 아래에 놓아야 한다.

---

## 3. 설치

### 3-1. 워크스페이스에 배치

이미 쓰는 워크스페이스가 있다면 그 `src/` 아래에 클론한다.

```bash
cd ~/scout2map_ws/src
git clone <this-repo> scout2map-bridge
```

워크스페이스가 없다면 먼저 만든다. `src` 디렉토리 하나만 있으면 된다.

```bash
mkdir -p ~/scout2map_ws/src
cd ~/scout2map_ws/src
git clone <this-repo> scout2map-bridge
```

결과 구조는 다음과 같다.

```
~/scout2map_ws/              ← 워크스페이스 루트. colcon은 여기서 실행한다
└── src/
    └── scout2map-bridge/    ← 이 레포
        ├── scout2map_msgs/
        └── scout2map_bridge/
```

레포가 한 겹 더 깊어도 상관없다. colcon은 `src/` 아래를 재귀적으로 훑어
`package.xml`이 있는 디렉토리를 모두 패키지로 인식한다.

### 3-2. 의존 패키지 설치

pyserial은 apt로 설치한다. pip로 넣으면 ROS2가 인식하지 못하는 경우가 있다.

```bash
sudo apt install python3-serial
```

### 3-3. 빌드

**반드시 워크스페이스 루트에서 실행한다.** `src/` 안에서 실행하면
`build`, `install`, `log` 디렉토리가 엉뚱한 위치에 생긴다.

```bash
cd ~/scout2map_ws
colcon build --symlink-install
```

`--symlink-install`을 붙이면 파이썬 파일이 복사가 아니라 심볼릭 링크로 설치되어,
코드를 고친 뒤 재빌드 없이 저장만 해도 반영된다.

빌드가 끝나면 다음처럼 나온다. 경고 없이 두 패키지 모두 `Finished`면 성공이다.

```
Starting >>> scout2map_msgs
Finished <<< scout2map_msgs [12.3s]
Starting >>> scout2map_bridge
Finished <<< scout2map_bridge [1.1s]

Summary: 2 packages finished [14.0s]
```

### 3-4. 환경 등록

빌드 결과를 쉘에 알려주는 단계다. **터미널을 새로 열 때마다 필요하다.**

```bash
source ~/scout2map_ws/install/setup.bash
```

매번 치기 번거로우면 `~/.bashrc` 끝에 넣어 둔다.

제대로 됐는지는 메시지 타입이 보이는지로 확인한다.
아래에서 필드 목록이 출력되면 `.msg` 컴파일과 환경 등록이 모두 성공한 것이다.

```bash
ros2 interface show scout2map_msgs/msg/EnvSnapshot
```

`Could not find the interface` 오류가 나면 source를 안 했거나 빌드가 실패한 것이다.

### 3-5. `.msg`를 수정했을 때

`.msg` 파일은 빌드 시점에 파이썬 클래스와 C++ 헤더로 변환된다.
그러므로 **정의를 고쳤다면 재빌드하고 다시 source해야 한다.**
이 단계를 건너뛰면 "필드가 없다"거나 타입이 맞지 않는다는 형태의,
원인을 찾기 어려운 오류가 난다.

```bash
cd ~/scout2map_ws
colcon build --packages-select scout2map_msgs
source install/setup.bash
```

---

## 4. 장치 경로 고정 (권장)

**MCU가 둘이므로 이 단계는 사실상 필수다.** 두 보드 모두 `/dev/ttyACM*`으로
잡히고 번호는 인식 순서를 따르므로, udev 규칙 없이는 부팅할 때마다
센서와 주행이 자리를 바꿀 수 있다. 기본 설정값이 `/dev/scout2map_pico`와
`/dev/scout2map_drive`인 것도 이 때문이다.

먼저 각 보드가 어떤 ID로 잡히는지 확인한다.

```bash
lsusb
```

두 MCU를 모두 붙였다면 두 항목이 보여야 한다.

| 장치 | VID:PID | 출처 |
|---|---|---|
| 센서 퓨전 MCU (Pico 2) | `2e8a:000a` | pico-sdk CDC 기본값 |
| 주행 제어 MCU (STM32) | `0483:5740` | `usb_cdc.c`의 디스크립터, ST 가상 COM 포트 |

값이 다르면 `udev/99-scout2map.rules` 안의 ID를 실제 값으로 고친다.

브릿지가 엉뚱한 보드를 열면 `parse_errors`나 `crc_errors`만 올라가는데,
증상이 배선 문제처럼 보여서 원인을 찾는 데 오래 걸린다.

규칙을 설치하고 즉시 적용한다.

```bash
sudo cp scout2map_bridge/udev/99-scout2map.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Pico 2를 뽑았다 다시 꽂은 뒤 심볼릭 링크가 생겼는지 확인한다.

```bash
ls -l /dev/scout2map_*
```

다음처럼 두 심볼릭 링크가 각각 다른 ttyACM을 가리키면 성공이다.

```
/dev/scout2map_drive -> ttyACM1
/dev/scout2map_pico  -> ttyACM0
```

마지막으로 시리얼 포트 접근 권한을 얻는다. **이 명령은 로그아웃 후 재로그인해야 적용된다.**

```bash
sudo usermod -aG dialout $USER
```

`groups` 명령 출력에 `dialout`이 보이면 적용된 것이다.

udev 설정을 건너뛰고 싶다면 실행할 때 실제 경로를 넘겨도 된다.

```bash
ros2 run scout2map_bridge sensor_bridge --ros-args -p port:=/dev/ttyACM0
```

---

## 5. 실행

Pico 2를 RPi5에 연결한 상태에서 실행한다.

두 MCU를 모두 붙였다면 한 번에 띄운다.

```bash
ros2 launch scout2map_bridge bringup.launch.py
```

한쪽만 붙어 있으면 해당 노드만 켠다.

```bash
ros2 launch scout2map_bridge bringup.launch.py drive:=false
ros2 launch scout2map_bridge sensor_bridge.launch.py    # 센서만, 개별 실행
ros2 launch scout2map_bridge drive_bridge.launch.py   # 주행만, 개별 실행
```

센서 브릿지가 정상이라면 콘솔에 다음 두 줄이 순서대로 나온다.

```
[INFO] [sensor_bridge]: serial opened: /dev/scout2map_pico
[INFO] [sensor_bridge]: sensor_bridge up: port=/dev/scout2map_pico frame_id=sensor_fusion snapshot=5.0Hz
```

MCU가 부팅 라인을 보내면 센서 초기화 결과도 함께 찍힌다.

```
[INFO] [sensor_bridge]: MCU boot: aht21=ok, ens160=ok, bh1750=ok
```

여기에 `FAIL`이 섞여 있으면 해당 센서의 배선이나 I2C 주소를 확인한다.
브릿지를 MCU보다 늦게 띄우면 이 줄을 놓칠 수 있는데, 그때는 MCU를 리셋하면 다시 나온다.

`serial open failed` 경고가 반복되면 아직 연결이 안 된 것이다.
노드는 죽지 않고 1초 간격으로 계속 재시도하므로, 케이블을 다시 꽂으면 알아서 붙는다.

주행 브릿지는 다음과 같이 나온다. `BOOT_INFO` 줄이 핵심이다.

```
[INFO] [drive_bridge]: serial opened: /dev/scout2map_drive
[INFO] [drive_bridge]: drive_bridge up: port=/dev/scout2map_drive cmd=20Hz timeout=0.25s limits=0.20m/s 0.80rad/s
[INFO] [drive_bridge]: MCU boot: fw 1.0.0, proto v1, 5764 counts/rev, track 240mm
```

`5764 counts/rev`가 나오면 펌웨어와 브릿지가 같은 구동계를 전제하고 있다는
뜻이다. 이 값이 다르면 오도메트리 전체가 비례 오차를 갖는다.

**주행 브릿지를 처음 띄울 때는 반드시 차체를 들어 바퀴를 띄운다.**
브릿지 README 20절의 브링업 순서를 따른다.

### 동작 확인

터미널을 새로 열고(여기서도 source가 필요하다) 데이터가 흐르는지 본다.

```bash
source ~/scout2map_ws/install/setup.bash
ros2 topic echo /sensors/status --once
```

`link_ok: true`면 MCU와 정상적으로 통신 중이다.
`false`라면 [브릿지 README의 트러블슈팅 표](scout2map_bridge/README.md#6-트러블슈팅)에서
증상별 원인을 찾는다.

스냅샷이 제 주기로 나오는지도 확인한다. 5Hz 근처면 정상이다.

```bash
ros2 topic hz /sensors/env_snapshot
ros2 topic echo /drive/status --once     # 주행부. link_ok와 estop_latched 확인
```

실제 값을 눈으로 보려면 다음을 쓴다.

```bash
ros2 topic echo /sensors/env_snapshot
```

---

## 6. 하드웨어 없이 개발하기

UGV가 한 대뿐이라 팀원이 돌아가며 실기를 쓸 수 없다.
그래서 **브릿지와 완전히 동일한 토픽·타입으로 가짜 값을 발행하는 노드**를 함께 넣었다.
구독하는 쪽 코드 입장에서는 진짜 하드웨어와 구분되지 않으므로,
이벤트 엔진이나 관제 서버를 Pico 2 없이 끝까지 개발할 수 있다.

```bash
ros2 launch scout2map_bridge fake_sensors.launch.py
```

실행하면 다음이 나온다.

```
[INFO] [fake_sensors]: fake_sensors up: scenario=normal (change it with: ros2 param set /fake_sensors scenario <name>)
[INFO] [fake_sensors]: available scenarios: normal, gas_leak, high_temp, low_light, dust_storm, warmup, sensor_dropout, link_loss
```

이 상태에서 7절의 모든 토픽이 실제 주기대로 발행된다.
**진짜 브릿지와 토픽 이름이 같으므로 `sensor_bridge`와 동시에 실행하면 안 된다.**
값이 뒤섞여 원인을 알 수 없는 동작을 하게 된다.
가짜 데이터임은 `/sensors/status`의 `port` 필드가 `SIMULATED`인 것으로 구분한다.

### 시나리오

값의 흐름을 시나리오로 바꾼다. 기본은 평온한 실내 상태(`normal`)다.

| 시나리오 | 무엇이 일어나는가 | 무엇을 시험하는가 |
|---|---|---|
| `normal` | 실내 평상값에 약간의 흔들림 | 평상시 오탐이 나지 않는지 |
| `gas_leak` | TVOC가 60 → 4500ppb, eCO2가 450 → 3200ppm으로 상승 | 가스 이벤트 발화와 해제 |
| `high_temp` | 온도가 24 → 62℃로 상승 | 고온 이벤트 |
| `low_light` | 조도가 320 → 8lux로 하락 | 저조도 이벤트 |
| `dust_storm` | PM2.5가 7 → 180µg/m³로 상승 | 먼지 관련 판정 |
| `warmup` | ENS160이 계속 `validity=1` | **워밍업 값을 걸러내는지** |
| `sensor_dropout` | ENS160만 발행 중단 | **`age_s` 증가와 `valid=false` 처리** |
| `link_loss` | 전 센서 발행 중단, `link_ok=false` | 통신 두절 이벤트 |

뒤의 세 개가 특히 중요하다. 정상 데이터만으로 짠 코드는 대개 여기서 무너진다.
`warmup`은 ENS160이 전원 인가 후 약 3분간 겪는 실제 상태이고,
`sensor_dropout`은 배선이 헐거워졌을 때 그대로 재현된다.
이때 `valid` 플래그를 확인하지 않은 코드는 **센서가 죽은 뒤에도 마지막 값으로 계속 이벤트를 낸다.**

### 실행 중에 시나리오 바꾸기

노드를 끄지 않고 파라미터만 바꾸면 된다.
값이 서서히 오르는 동안 이벤트 엔진이 어느 지점에서 발화하는지 눈으로 볼 수 있다.

```bash
ros2 param set /fake_sensors scenario gas_leak
ros2 param set /fake_sensors scenario normal      # 되돌리기
```

시나리오를 바꾸면 상승 곡선이 처음부터 다시 시작된다.
기본 상승 시간은 30초이며, 빨리 보고 싶으면 줄인다.

```bash
ros2 launch scout2map_bridge fake_sensors.launch.py scenario:=gas_leak ramp_seconds:=5.0
```

임계값을 정확히 맞춰 시험할 때는 흔들림을 꺼서 값을 매끄럽게 만든다.

```bash
ros2 launch scout2map_bridge fake_sensors.launch.py noise:=0.0
```

### 값이 나오는지 확인

```bash
ros2 topic echo /sensors/env_snapshot
```

`sensor_dropout`으로 바꾼 뒤 같은 명령을 보면
`air_quality_age_s`가 계속 커지고 `air_quality_valid`가 false로 바뀐다.
이 동작이 실제 하드웨어에서 센서가 죽었을 때와 동일하다.

### 개발 순서 제안

1. `fake_sensors`로 노드를 만들고 시나리오별로 동작을 확인한다.
2. 특히 `warmup`, `sensor_dropout`, `link_loss`에서 오작동하지 않는지 본다.
3. 팀장에게 코드를 넘겨 실기에서 검증한다.

1과 2를 충분히 하면 실기 검증에서 잡을 것이 거의 남지 않는다.
반대로 이 단계를 건너뛰면 한 대뿐인 하드웨어 앞에서 기본적인 버그를 잡게 된다.

---


## 7. 발행 토픽

| 토픽 | 타입 | 주기 |
|---|---|---|
| `/sensors/env_snapshot` | `scout2map_msgs/EnvSnapshot` | 5Hz |
| `/sensors/temperature` | `sensor_msgs/Temperature` | 1Hz |
| `/sensors/humidity` | `sensor_msgs/RelativeHumidity` | 1Hz |
| `/sensors/illuminance` | `sensor_msgs/Illuminance` | 5Hz |
| `/sensors/air_quality` | `scout2map_msgs/AirQuality` | 1Hz |
| `/sensors/particulate` | `scout2map_msgs/Particulate` | ~1Hz |
| `/sensors/status` | `scout2map_msgs/SensorStatus` | 1Hz |
| `/sensors/raw_json` | `std_msgs/String` | 옵션 |

주행 제어 MCU(STM32) 쪽은 다음과 같다. 구독은 `/cmd_vel` 하나다.

| 토픽 | 타입 | 주기 |
|---|---|---|
| `/drive/odom` | `nav_msgs/Odometry` | 50Hz |
| `/drive/imu` | `sensor_msgs/Imu` | 50Hz |
| `/drive/range` | `sensor_msgs/Range` | 50Hz |
| `/drive/battery` | `sensor_msgs/BatteryState` | 50Hz |
| `/drive/status` | `scout2map_msgs/DriveStatus` | 10Hz |
| `/drive/diagnostics` | `scout2map_msgs/DriveDiagnostics` | 요청 시 |

서비스는 `/drive/estop`, `/drive/clear_fault`, `/drive/reset_odom`,
`/drive/request_diagnostics`이며 모두 `std_srvs/Trigger`다.

**이벤트 엔진을 만든다면 `/sensors/env_snapshot` 하나만 구독하면 된다.**
센서마다 발행 주기가 다르기 때문에 개별 토픽을 직접 조인하면
"온도는 방금 값인데 가스는 3초 전 값"인 상태로 임계값을 판단하게 된다.
스냅샷은 전 센서의 최신값을 값의 나이·유효 플래그와 함께 묶어 발행하므로 이 문제가 없다.

각 필드가 정확히 무엇을 뜻하는지는 [`scout2map_msgs/README.md`](scout2map_msgs/README.md)를 본다.
노드 자체의 동작과 파라미터는 [`scout2map_bridge/README.md`](scout2map_bridge/README.md)에 있다.

---

## 8. 다른 워크스페이스에서 메시지만 쓰고 싶다면

이벤트 엔진처럼 브릿지 없이 메시지 타입만 필요한 경우에도
현재는 이 레포를 통째로 클론해 두 패키지를 함께 빌드하면 된다.
브릿지 노드를 실행하지 않으면 그만이다.

---

## 9. v1.0.0에서 올라올 때 (이름 변경)

주행 브릿지가 들어오면서 이름 기준을 **역할 기반으로 통일했다.**
기존에는 `pico_bridge`(보드 이름)와 `drive_bridge`(역할)가 섞여 있었다.
보드 기준 이름은 하드웨어를 교체하면 거짓말이 되므로 역할 쪽으로 맞췄다.

| v1.0.0 | 현재 |
|---|---|
| 실행 파일 `pico_bridge` | `sensor_bridge` |
| 노드 이름 `/pico_bridge` | `/sensor_bridge` |
| `pico_bridge.launch.py` | `sensor_bridge.launch.py` |
| `config/pico_bridge.yaml` | `config/sensor_bridge.yaml` |
| 토픽 `/bridge/status` | `/sensors/status` |
| 메시지 `BridgeStatus` | `SensorStatus` |

`/sensors/*`와 `/drive/*`로 네임스페이스가 갈리고,
`SensorStatus`와 `DriveStatus`가 짝을 이룬다.

**구독하는 쪽에서 고칠 것은 두 가지다.** 나머지 센서 토픽은 그대로다.

```python
# 이전
from scout2map_msgs.msg import BridgeStatus
self.create_subscription(BridgeStatus, '/bridge/status', cb, 10)

# 현재
from scout2map_msgs.msg import SensorStatus
self.create_subscription(SensorStatus, '/sensors/status', cb, 10)
```

`/sensors/env_snapshot`을 비롯한 환경 센서 토픽은 이름도 필드도 바뀌지 않았다.
이벤트 엔진이 스냅샷만 구독하고 있다면 고칠 것이 없다.

`SensorStatus`에 `framing_overflows` 필드가 하나 추가되었다.
개행 없이 바이트가 계속 쌓여 버퍼를 비운 횟수이며, 정상 동작 시 0이다.

---

## 10. 커밋하지 않는 것

이 레포는 워크스페이스가 아니므로 `build/`, `install/`, `log/`가 여기에 생기지 않는다.
그 디렉토리들은 워크스페이스 루트(`~/scout2map_ws/`)에 생기며, 그쪽에서 제외한다.

```gitignore
__pycache__/
*.pyc
*.egg-info/
.vscode/
```
