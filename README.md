# scout2map-bridge

Scout2Map UGV의 **센서 퓨전 MCU(Raspberry Pi Pico 2)와 ROS2를 연결하는 브릿지**다.
RPi5에서 동작하며, ROS2 패키지 두 개로 구성된다.

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

관련 레포는 다음과 같이 나뉜다. 경계는 "어느 하드웨어에서 실행되는가"이다.

| 레포 | 실행 위치 | 빌드 체계 |
|---|---|---|
| Pico 2 펌웨어 | Pico 2 | pico-sdk |
| STM32 펌웨어 | STM32 | arm-none-eabi-gcc + Makefile |
| **이 레포** | **RPi5** | **colcon (ROS2)** |

주행 제어 MCU(STM32)용 브릿지 노드도 RPi5에서 도는 코드이므로 나중에 이 레포에 추가된다.

---

## 2. 구성

```
scout2map-bridge/
├── scout2map_msgs/       # 메시지 타입 정의. 실행되는 노드는 없다
│   ├── msg/              #   AirQuality, Particulate, EnvSnapshot, BridgeStatus
│   └── README.md         #   ★ 필드별 상세 레퍼런스
└── scout2map_bridge/     # 브릿지 노드 본체
    ├── scout2map_bridge/pico_bridge_node.py
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

Pico 2는 `/dev/ttyACM0`으로 잡히지만, 다른 USB 시리얼 장치가 함께 물리면
부팅 순서에 따라 번호가 뒤바뀐다. udev 규칙으로 고정 경로를 만들어 두면
이 문제가 사라진다. 기본 설정값이 `/dev/scout2map_pico`인 것도 이 때문이다.

먼저 Pico 2가 어떤 ID로 잡히는지 확인한다.

```bash
lsusb
```

출력에서 `2e8a:000a` 같은 항목을 찾는다. `2e8a`는 Raspberry Pi의 벤더 ID이고,
`000a`는 pico-sdk가 USB CDC에 쓰는 기본 제품 ID다.
값이 다르면 `udev/99-scout2map-pico.rules` 파일 안의 ID를 실제 값으로 고친다.

규칙을 설치하고 즉시 적용한다.

```bash
sudo cp scout2map_bridge/udev/99-scout2map-pico.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Pico 2를 뽑았다 다시 꽂은 뒤 심볼릭 링크가 생겼는지 확인한다.

```bash
ls -l /dev/scout2map_pico
```

`/dev/scout2map_pico -> ttyACM0` 형태로 나오면 성공이다.

마지막으로 시리얼 포트 접근 권한을 얻는다. **이 명령은 로그아웃 후 재로그인해야 적용된다.**

```bash
sudo usermod -aG dialout $USER
```

`groups` 명령 출력에 `dialout`이 보이면 적용된 것이다.

udev 설정을 건너뛰고 싶다면 실행할 때 실제 경로를 넘겨도 된다.

```bash
ros2 run scout2map_bridge pico_bridge --ros-args -p port:=/dev/ttyACM0
```

---

## 5. 실행

Pico 2를 RPi5에 연결한 상태에서 실행한다.

```bash
ros2 launch scout2map_bridge pico_bridge.launch.py
```

정상이라면 콘솔에 다음 두 줄이 순서대로 나온다.

```
[INFO] [pico_bridge]: serial opened: /dev/scout2map_pico
[INFO] [pico_bridge]: pico_bridge up: port=/dev/scout2map_pico frame_id=sensor_fusion snapshot=5.0Hz
```

MCU가 부팅 라인을 보내면 센서 초기화 결과도 함께 찍힌다.

```
[INFO] [pico_bridge]: MCU boot: aht21=ok, ens160=ok, bh1750=ok
```

여기에 `FAIL`이 섞여 있으면 해당 센서의 배선이나 I2C 주소를 확인한다.
브릿지를 MCU보다 늦게 띄우면 이 줄을 놓칠 수 있는데, 그때는 MCU를 리셋하면 다시 나온다.

`serial open failed` 경고가 반복되면 아직 연결이 안 된 것이다.
노드는 죽지 않고 1초 간격으로 계속 재시도하므로, 케이블을 다시 꽂으면 알아서 붙는다.

### 동작 확인

터미널을 새로 열고(여기서도 source가 필요하다) 데이터가 흐르는지 본다.

```bash
source ~/scout2map_ws/install/setup.bash
ros2 topic echo /bridge/status --once
```

`link_ok: true`면 MCU와 정상적으로 통신 중이다.
`false`라면 [브릿지 README의 트러블슈팅 표](scout2map_bridge/README.md#6-트러블슈팅)에서
증상별 원인을 찾는다.

스냅샷이 제 주기로 나오는지도 확인한다. 5Hz 근처면 정상이다.

```bash
ros2 topic hz /sensors/env_snapshot
```

실제 값을 눈으로 보려면 다음을 쓴다.

```bash
ros2 topic echo /sensors/env_snapshot
```

---

## 6. 발행 토픽

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
센서마다 발행 주기가 다르기 때문에 개별 토픽을 직접 조인하면
"온도는 방금 값인데 가스는 3초 전 값"인 상태로 임계값을 판단하게 된다.
스냅샷은 전 센서의 최신값을 값의 나이·유효 플래그와 함께 묶어 발행하므로 이 문제가 없다.

각 필드가 정확히 무엇을 뜻하는지는 [`scout2map_msgs/README.md`](scout2map_msgs/README.md)를 본다.
노드 자체의 동작과 파라미터는 [`scout2map_bridge/README.md`](scout2map_bridge/README.md)에 있다.

---

## 7. 다른 워크스페이스에서 메시지만 쓰고 싶다면

이벤트 엔진처럼 브릿지 없이 메시지 타입만 필요한 경우에도
현재는 이 레포를 통째로 클론해 두 패키지를 함께 빌드하면 된다.
브릿지 노드를 실행하지 않으면 그만이다.

인터페이스가 안정된 뒤에는 `scout2map_msgs`를 별도 레포로 분리하는 편이 낫다.
지금 분리하면 필드 하나를 고칠 때마다 두 레포를 함께 커밋해야 해서 오히려 번거롭다.

---

## 8. 커밋하지 않는 것

이 레포는 워크스페이스가 아니므로 `build/`, `install/`, `log/`가 여기에 생기지 않는다.
그 디렉토리들은 워크스페이스 루트(`~/scout2map_ws/`)에 생기며, 그쪽에서 제외한다.

```gitignore
__pycache__/
*.pyc
*.egg-info/
.vscode/
```
