# scout2map-bridge

Scout2Map UGV의 **센서 퓨전 MCU(Raspberry Pi Pico 2)와 ROS2를 연결하는 브릿지** 레포다.
이 레포 하나가 곧 colcon 워크스페이스이며, 클론한 자리에서 바로 빌드된다.

---

## 1. 시스템 내 위치

```
[ 센서들 ]                    [ 센서 퓨전 MCU ]        [ RPi5 / ROS2 ]
AHT21   ─ I2C1 ┐
ENS160  ─ I2C1 ┤
BH1750  ─ I2C0 ┼──────────▶  Raspberry Pi Pico 2  ──▶  pico_bridge  ──▶  /sensors/*
PMS7003 ─ UART ┘              (JSON per line)          (이 레포)          이벤트 엔진
                                                                          맵 마커 노드
                                                                          관제 서버
```

MCU는 센서를 각자의 주기로 읽어 JSON 한 줄씩 USB CDC로 내보낸다.
브릿지는 그 라인을 파싱해 ROS2 토픽으로 재발행한다.
**이 레포의 목적은 다른 파트가 시리얼도 JSON도 몰라도 되게 만드는 것**이다.

주행 제어 MCU(STM32)는 프로토콜이 다르고 양방향이므로 별도 레포·별도 노드로 만든다.

---

## 2. 레포 구조

```
scout2map-bridge/
├── src/
│   ├── scout2map_msgs/      # 메시지 타입 정의. 실행되는 노드 없음
│   │   ├── msg/             #   AirQuality, Particulate, EnvSnapshot, BridgeStatus
│   │   └── README.md        #   ★ 필드별 상세 레퍼런스 (다른 파트는 이 문서를 본다)
│   └── scout2map_bridge/    # 브릿지 노드 본체
│       ├── scout2map_bridge/pico_bridge_node.py
│       ├── config/          #   파라미터 YAML
│       ├── launch/
│       ├── udev/            #   장치 경로 고정 규칙
│       └── README.md        #   ★ 노드 동작 방식, 파라미터, 트러블슈팅
├── README.md                # 이 문서
└── .gitignore
```

`src/` 한 겹은 colcon의 요구사항이다. colcon은 `src/` 아래를 재귀 탐색해 패키지를 찾는다.

패키지가 둘로 나뉜 이유는 `.msg` 컴파일이 CMake 기반이라 파이썬 패키지 안에서 돌지 않기 때문이고,
동시에 의존성 측면에서도 옳다. 이벤트 엔진은 `scout2map_msgs`만 있으면 되고
브릿지 코드나 pyserial까지 끌어올 이유가 없다.

---

## 3. 빌드

```bash
git clone <this-repo> scout2map-bridge
cd scout2map-bridge

sudo apt install python3-serial
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install`을 붙이면 파이썬 파일은 저장만 해도 반영되어 재빌드가 필요 없다.
단 **`.msg` 파일을 수정했을 때는 반드시 재빌드하고 다시 source한다.**
코드 생성이 다시 돌아야 하며, 이를 건너뛰면 "필드가 없다"는 형태의
원인 찾기 어려운 오류가 난다.

```bash
colcon build --packages-select scout2map_msgs && source install/setup.bash
```

---

## 4. 실행

장치 경로를 먼저 고정한다. 그러지 않으면 `/dev/ttyACM0`과 `/dev/ttyACM1`이
부팅 순서에 따라 뒤바뀐다.

```bash
lsusb | grep -i raspberry     # VID:PID 확인 (pico-sdk 기본값 2e8a:000a)
sudo cp src/scout2map_bridge/udev/99-scout2map-pico.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout $USER    # 로그아웃 후 재로그인해야 적용된다
```

띄운다.

```bash
ros2 launch scout2map_bridge pico_bridge.launch.py
```

살아 있는지 확인한다.

```bash
ros2 topic echo /bridge/status --once     # link_ok가 true인지 본다
ros2 topic hz /sensors/env_snapshot       # 5Hz 근처면 정상
```

---

## 5. 발행 토픽 한눈에 보기

| 토픽 | 타입 | 주기 |
|---|---|---|
| `/sensors/env_snapshot` | `scout2map_msgs/EnvSnapshot` | 5Hz |
| `/sensors/temperature` | `sensor_msgs/Temperature` | 1Hz |
| `/sensors/humidity` | `sensor_msgs/RelativeHumidity` | 1Hz |
| `/sensors/illuminance` | `sensor_msgs/Illuminance` | 5Hz |
| `/sensors/air_quality` | `scout2map_msgs/AirQuality` | 1Hz |
| `/sensors/particulate` | `scout2map_msgs/Particulate` | ~1Hz |
| `/bridge/status` | `scout2map_msgs/BridgeStatus` | 1Hz |
| `/sensors/raw_json` | `std_msgs/String` | 옵션 |

**이벤트 엔진을 만든다면 `/sensors/env_snapshot` 하나만 구독하면 된다.**
센서마다 주기가 다르기 때문에 개별 토픽을 직접 조인하면
"온도는 방금 값인데 가스는 3초 전 값"인 상태로 임계값을 판단하게 된다.
스냅샷은 전 센서의 최신값을 값의 나이·유효 플래그와 함께 묶어 발행하므로 이 문제가 없다.

각 필드가 정확히 무엇을 뜻하는지는 [`src/scout2map_msgs/README.md`](src/scout2map_msgs/README.md)를 본다.
노드 자체의 동작과 파라미터는 [`src/scout2map_bridge/README.md`](src/scout2map_bridge/README.md)에 있다.

---

## 6. 관련 레포

| 레포 | 내용 |
|---|---|
| (이 레포) | ROS2 브릿지 노드 + 메시지 정의 |
| Pico 2 펌웨어 | 센서 퓨전 MCU 베어메탈 C. 이 브릿지가 파싱하는 JSON을 생성한다 |
| STM32 펌웨어 | 주행 제어 MCU 베어메탈 C |

MCU 펌웨어의 출력 포맷을 바꾸면 브릿지의 파서도 함께 고쳐야 한다.
센서를 추가했는데 브릿지를 고치지 않으면 `BridgeStatus.unknown_src` 카운터가 올라간다.

---

## 7. 커밋하지 않는 것

```gitignore
build/
install/
log/
__pycache__/
*.pyc
```

colcon 빌드 산출물은 용량이 크고 환경마다 다르므로 반드시 제외한다.
