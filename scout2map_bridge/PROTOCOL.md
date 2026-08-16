# PROTOCOL.md — STM32 주행 제어 MCU 와이어 포맷

`S2M-FW-DrivingControl`의 `lib/control/protocol.h`가 **단일 진실 공급원**이며,
이 문서는 그것을 반영한다. 펌웨어 헤더가 바뀌면 이 문서와
`scout2map_bridge/drive_protocol.py`를 함께 갱신한다.

기준 펌웨어: `PROTO_VERSION = 1`

---

## 1. 프레임 구조

```
[0]     0xAA          sync high
[1]     0x55          sync low
[2]     TYPE          메시지 종류
[3]     LEN           페이로드 길이 (헤더와 CRC 제외)
[4..]   PAYLOAD       LEN 바이트, 리틀 엔디언
[..+2]  CRC16         TYPE/LEN/PAYLOAD에 대한 CCITT, 빅 엔디언
```

| 상수 | 값 |
|---|---|
| `PROTO_MAX_PAYLOAD` | 56 |
| `PROTO_MAX_FRAME` | 62 |

최대 프레임 62바이트는 64바이트 벌크 전송 1회에 들어간다.
따라서 호스트 측 리더가 프레임을 재조립할 필요가 없다.

### 함정 하나 — CRC만 빅 엔디언이다

헤더 주석은 "모든 다중 바이트 필드는 리틀 엔디언"이라고 적고 있으나
**CRC 자체는 상위 바이트를 먼저 보낸다.** `framing.c`가 명시적으로 그렇게 쓴다.

```c
out[PROTO_HEADER_LEN + len]      = (uint8_t)(crc >> 8);
out[PROTO_HEADER_LEN + len + 1U] = (uint8_t)(crc & 0xFFU);
```

리틀 엔디언으로 읽으면 **모든 프레임이 CRC 불일치로 거부된다.**
파이썬에서는 `struct.unpack(">H", ...)`를 쓴다.

### CRC16-CCITT

다항식 0x1021, 초기값 0xFFFF. sync 바이트는 포함하지 않는다.

```python
def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc
```

---

## 2. 메시지 종류

| TYPE | 방향 | 이름 | 페이로드 |
|---|---|---|---|
| 0x01 | 호스트 → MCU | `CMD_VELOCITY` | 4B |
| 0x02 | 호스트 → MCU | `CMD_WHEEL_RAW` | 4B |
| 0x03 | 호스트 → MCU | `CMD_ESTOP` | 0B |
| 0x04 | 호스트 → MCU | `CMD_RESET_ODOM` | 0B |
| 0x05 | 호스트 → MCU | `CMD_CLEAR_FAULT` | 0B |
| 0x06 | 호스트 → MCU | `CMD_PING` | 0B |
| 0x07 | 호스트 → MCU | `CMD_DIAG` | 0B |
| 0x08 | 호스트 → MCU | `CMD_I2C_SCAN` | 0B |
| 0x81 | MCU → 호스트 | `TELEMETRY` | **56B**, 50Hz |
| 0x86 | MCU → 호스트 | `PONG` | 0B |
| 0x87 | MCU → 호스트 | `BOOT_INFO` | 8B, 포트 오픈 시 1회 |
| 0x88 | MCU → 호스트 | `DIAG` | **24B**, 요청 시 |
| 0x89 | MCU → 호스트 | `I2C_SCAN` | **18B**, 요청 시 |

### 함정 둘 — 헤더의 크기 주석이 낡아 있다

`protocol.h`의 구조체 위 주석은 TELEMETRY를 44바이트, DIAG를 16바이트,
I2C_SCAN을 17바이트로 적고 있으나, **구조체 필드 목록을 실제로 계산하면
각각 56, 24, 18바이트다.** 필드 목록이 정본이고 주석이 낡은 것이다.

56바이트는 `PROTO_MAX_PAYLOAD`와 정확히 일치하며, 프레임 전체가 62바이트로
`PROTO_MAX_FRAME`에 들어맞는다는 점이 이를 뒷받침한다.
펌웨어의 호스트 도구도 24바이트 DIAG를 기대한다.

**따라서 이 숫자들을 코드에 상수로 박지 않는다.** `struct.calcsize()`로
유도하여, 펌웨어가 바뀌었을 때 조용한 오파싱 대신 명확한 버전 오류가
나도록 한다.

---

## 3. 스케일링

물리량은 부동소수점이 아닌 스케일된 정수로 전송한다.
MCU에 FPU가 없고, 고정소수점이 프레임 크기와 호스트 파싱 양쪽에 유리하다.

| 물리량 | 단위 | 타입 | 변환 |
|---|---|---|---|
| 선속도 | mm/s | int16 | ÷1000 → m/s |
| 각속도 | mrad/s | int16 | ÷1000 → rad/s |
| 듀티 | permille | int16 | −1000 ~ 1000 |
| 위치 | mm | int32 | ÷1000 → m |
| 방위 | mrad | int32 | ÷1000 → rad |
| 쿼터니언 | 1/16384 | int16 | ×(1/16384), BNO055 원본 |
| 자이로 | 1/16 deg/s | int16 | ×(1/16)×(π/180) → rad/s |
| 가속도 | 1/100 m/s² | int16 | ÷100, **중력 포함** |
| 거리 | mm | uint16 | 센티널 주의, 5절 참조 |
| 배터리 | mV | uint16 | ÷1000 → V |
| 타임스탬프 | ms | uint32 | 약 49.7일마다 랩어라운드 |

---

## 4. TELEMETRY (0x81), 56바이트

`struct` 포맷: `<IiihhiiihhhhhhhhHHhhHBB`

| 오프셋 | 필드 | 타입 | 설명 |
|---|---|---|---|
| 0 | `timestamp_ms` | uint32 | MCU 가동 시간, 샘플 시점에 찍힘 |
| 4 | `enc_left` | int32 | 부팅 후 누적 카운트 |
| 8 | `enc_right` | int32 | |
| 12 | `speed_left_mmps` | int16 | 측정된 휠 표면 속도 |
| 14 | `speed_right_mmps` | int16 | |
| 16 | `odom_x_mm` | int32 | 적분된 자세 |
| 20 | `odom_y_mm` | int32 | |
| 24 | `odom_theta_mrad` | int32 | |
| 28 | `quat_w` | int16 | BNO055 융합 출력 |
| 30 | `quat_x` | int16 | |
| 32 | `quat_y` | int16 | |
| 34 | `quat_z` | int16 | |
| 36 | `gyro_z` | int16 | 요레이트. 슬립 검출이 필요로 하는 항 |
| 38 | `accel_x` | int16 | |
| 40 | `accel_y` | int16 | |
| 42 | `accel_z` | int16 | |
| 44 | `distance_mm` | uint16 | 5절 참조 |
| 46 | `battery_mv` | uint16 | ADC 미보고 시 0 |
| 48 | `duty_left` | int16 | 인가된 듀티 |
| 50 | `duty_right` | int16 | |
| 52 | `status` | uint16 | 비트필드, 6절 참조 |
| 54 | `imu_calib` | uint8 | 패킹된 캘리브레이션 |
| 55 | `reserved` | uint8 | 4바이트 정렬 유지용 |

**`gyro_z`만 전송된다.** 롤/피치 각속도는 와이어에 없으므로,
`sensor_msgs/Imu`로 옮길 때 해당 성분의 공분산을 크게 잡아야 한다.

**`duty_left`/`duty_right`는 모터 반전 적용 전의 논리 듀티다.**
전진 명령 시 양쪽 모두 양수로 표시된다. 전기적 부호는 모터 장착 방향에
따른 구현 세부사항이다.

### `imu_calib` 언패킹

BNO055 캘리브레이션 4종이 한 바이트에 들어 있다. 0이 미완료, 3이 완료다.

```
bits 7:6  sys
bits 5:4  gyro
bits 3:2  accel
bits 1:0  mag
```

---

## 5. 거리 센티널

`distance_mm`은 두 개의 특수값을 갖는다. **둘 다 "숫자가 없다"는 뜻이지만
이유가 정반대이고, 코스트맵은 이를 다르게 다뤄야 한다.**

| 값 | 의미 | 브릿지 처리 |
|---|---|---|
| 40 ~ 300 | 측정 거리 (mm) | 그대로 미터 변환 |
| 0xFFFE | 최소 거리 이내, 거리 불명 | `range = min_range` |
| 0xFFFF | 측정 범위 밖 | `range = +inf` |

0xFFFE를 무한대로 처리하면 **바로 앞의 장애물이 빈 공간으로 기록된다.**
거리는 모르지만 장애물이 있다는 것은 확실하므로, 측정 가능한 최소값으로
보고하는 것이 안전한 해석이다.

GP2D120X는 약 4cm 미만에서 출력 전압이 다시 하강하여 2.0V가 5cm일 수도
2cm일 수도 있다. 펌웨어가 이 모호 구간을 래치하여 0xFFFE로 보고한다.

---

## 6. 상태 비트필드

| 비트 | 이름 | 의미 |
|---|---|---|
| 0 | `MOTOR_ENABLED` | |
| 1 | `OPENLOOP` | 엔코더 피드백 유실, 개루프 폴백 중 |
| 2 | `FAULT_STALL` | |
| 3 | `CMD_TIMEOUT` | 호스트 무응답으로 펌웨어가 정지시킴 |
| 4 | `ESTOP_LATCHED` | |
| 5 | `IMU_OK` | |
| 6 | `BATT_WARN` | |
| 7 | `BATT_CRITICAL` | |
| 8 | `IMU_CALIBRATED` | 융합 서브시스템 완전 캘리브레이션 |
| 9 | `BATT_DEAD` | 펌웨어가 구동을 차단함 |

**`OPENLOOP`이 서면 오도메트리를 믿을 수 없다.** 브릿지는 이때
`Odometry`의 공분산을 100배로 부풀려 하위 필터가 스스로 알아내도록
방치하지 않는다.

**`IMU_CALIBRATED`가 서기 전에는 절대 방위가 드리프트한다.**
상대적 자세 변화는 사용 가능하므로, 브릿지는 방위 공분산만 키운다.

배터리 정책은 계층이 갈린다. `WARN`과 `CRITICAL`은 **보고만** 하며 대응은
SBC 몫이다. 복귀와 데이터 백업이 모두 주행을 필요로 하므로, 펌웨어가
임의로 차단하면 상위 노드의 정책 자체가 불가능해진다.
`DEAD`만 예외로 펌웨어가 구동을 차단한다. 셀당 3.0V 미만에서 스톨 전류를
계속 인출하는 것은 용량 문제가 아니라 발화 위험이기 때문이다.

---

## 7. BOOT_INFO (0x87), 8바이트

`struct` 포맷: `<BBBBHH`

| 필드 | 타입 | 설명 |
|---|---|---|
| `proto_version` | uint8 | 불일치 시 브릿지가 경고 |
| `fw_major` | uint8 | |
| `fw_minor` | uint8 | |
| `fw_patch` | uint8 | |
| `counts_per_wheel_rev` | uint16 | 실측 5764 (11 PPR × 4 × 131) |
| `wheel_base_mm` | uint16 | **실제로는 트랙 폭**, 240mm |

포트 오픈 직후 1회 전송된다. 호스트가 오도메트리 스케일을 하드코딩하지
않아도 되며, 모터를 교체하면 브릿지가 자동으로 따라온다.

브릿지는 이 값이 도착하면 파라미터의 트랙 폭을 덮어쓴다.
펌웨어가 실제 빌드를 알고 있고 파라미터는 추정값이기 때문이다.

---

## 8. DIAG (0x88), 24바이트

`struct` 포맷: `<BBBBIIHHHHHH`

| 필드 | 타입 | 설명 |
|---|---|---|
| `imu_init_step` | uint8 | 초기화 상태머신이 멈춘 지점 |
| `imu_chip_id` | uint8 | BNO055 응답 시 0xA0, 무응답 시 0xFF |
| `imu_calib` | uint8 | 패킹된 sys/gyr/acc/mag |
| `reserved` | uint8 | |
| `imu_read_ok` | uint32 | |
| `imu_read_fail` | uint32 | |
| `i2c_errors` | uint16 | |
| `i2c_recoveries` | uint16 | 버스를 수동으로 풀어준 횟수 |
| `batt_counts` | uint16 | 원시 ADC |
| `batt_mv` | uint16 | |
| `dist_counts` | uint16 | 응답 곡선 적용 전 |
| `dist_mv` | uint16 | 중앙값 필터 적용 후 |

원시 카운트가 함께 오는 이유는 재보정 때문이다. 보정 대상 상수가
자기 자신의 계산에 개입하면 안 되므로 스케일된 값이 아닌 카운트가 필요하다.

---

## 9. I2C_SCAN (0x89), 18바이트

`struct` 포맷: `<BB16s`

| 필드 | 타입 | 설명 |
|---|---|---|
| `count` | uint8 | 응답한 장치 수 |
| `lines` | uint8 | bit0 SCL idle high, bit1 SDA idle high |
| `bitmap` | 16B | 7비트 주소당 1비트, LSB first |

주소 `a`의 응답 여부는 `bitmap[a // 8] & (1 << (a % 8))`이다.
목록이 아닌 비트맵인 이유는 응답 수와 무관하게 페이로드 크기를
고정하기 위해서다.

`lines`가 함께 오므로 풀업 부재와 센서 무응답을 구분할 수 있다.

---

## 10. 명령 규약

**명령은 ACK되지 않는다.** 호스트가 `cmd_vel`을 연속 발행하므로 유실된
명령은 다음 주기에 자동 복구된다. 실제 링크 단절은 펌웨어의 명령
타임아웃(300ms)이 처리한다.

브릿지는 이 300ms를 **정상 경로가 아닌 최후의 방어선**으로 취급한다.
ROS 측이 조용해지면 브릿지가 먼저 명시적으로 0을 보낸다.
펌웨어 타임아웃은 브릿지 자체가 죽었을 때를 위한 것이다.

**텔레메트리는 큐잉되지 않고 폐기된다.** 이전 패킷이 전송 중이면 해당
프레임을 버린다. 지연 도착한 오래된 값은 누락보다 해롭고, 다음 프레임이
20ms 뒤에 온다.

**E-stop은 래치된다.** 이후의 속도 명령으로 해제되지 않으며
`CMD_CLEAR_FAULT`를 명시적으로 보내야 복귀한다.

**`CMD_WHEEL_RAW`는 PID를 우회한다.** 브링업 시 배선/기어 문제와 제어
튜닝 문제를 분리하기 위한 경로이며, 일반 속도 명령을 받으면 자동 해제된다.
브릿지는 이 명령을 노출하지 않는다. 바닥에 놓인 상태로 실행하면 즉시
주행하므로, 펌웨어 저장소의 `tools/s2m_console.py --raw`를 쓰는 편이 안전하다.

---

## 11. 디코더 요구사항

USB 바이트가 유실되면 이후 모든 프레임이 밀린다. 디코더는 임의의 쓰레기
바이트열에서 스스로 동기를 되찾아야 하며, 잘못된 길이 필드에서 영구히
대기해서는 안 된다.

1. sync 쌍(`0xAA 0x55`)을 탐색한다.
2. TYPE이 알려진 값인지, LEN이 56 이하인지 **CRC 계산 전에** 검사한다.
   이 검사가 없으면 잘못된 길이를 믿고 오지 않을 바이트를 기다리게 된다.
3. CRC 불일치 시 sync 2바이트만 버린다. 진짜 프레임이 페이로드로
   오인한 구간 안에서 시작될 수 있기 때문이다.
4. 페이로드 안에 `0xAA 0x55`가 데이터로 나타날 수 있다. 길이 기반
   파싱이므로 문제되지 않으나, 바이트 스터핑이 없다는 점은 인지한다.

`drive_protocol.py`의 `FrameDecoder`가 이를 구현하며, 다음이 검증되었다.

- 선행 쓰레기 + 프레임 중간 분할 + 후행 부분 sync
- 1바이트씩 전달 (최악의 USB 단편화)
- CRC 손상 후 다음 정상 프레임 복구
- 페이로드에 sync 패턴이 포함된 프레임
- 12KB 무작위 노이즈 후 정상 프레임 검출
- 과대 LEN 주장 거부

---

## 12. 검증 상태

`drive_protocol.py`는 펌웨어 저장소의 `tools/s2m_console.py`와 다음이
바이트 단위로 일치함을 확인했다.

| 항목 | 결과 |
|---|---|
| `crc16` 구현 | 일치 |
| `encode_velocity` 출력 | 일치 |
| `encode_wheel_raw` 출력 | 일치 |
| 무페이로드 프레임 인코딩 | 일치 |
| TELEMETRY / BOOT_INFO / DIAG 포맷 문자열 | 일치 |

**실물 STM32와의 통신은 아직 검증되지 않았다.** 위 확인은 두 호스트 측
구현이 서로 일치한다는 것이며, 실제 보드에서 프레임이 흐르는 것을
확인한 것은 아니다.
