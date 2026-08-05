# scout2map_bridge

센서 퓨전 MCU(Raspberry Pi Pico 2)가 USB CDC로 내보내는 JSON 라인을 파싱해
ROS2 토픽으로 재발행하는 브릿지 노드다.

이 문서는 **노드 자체를 다루거나 고치는 사람**을 위한 것이다.
토픽에 어떤 값이 들어오는지 알고 싶을 뿐이라면
[`scout2map_msgs/README.md`](../scout2map_msgs/README.md)를 본다.

---

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
[USB CDC] ──▶ SerialReader (별도 스레드)
                   │  \n 단위로 잘라 (수신시각, 라인) 튜플 생성
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

값은 `config/pico_bridge.yaml`에서 수정하는 것이 기본이다.
런치 파일이 이 YAML을 읽어 노드에 넘긴다.

일회성으로 바꿔 볼 때는 `ros2 run`에 직접 넘긴다.
다만 이 경우 YAML은 읽히지 않으므로, 명시하지 않은 값은 코드의 기본값이 쓰인다.

```bash
ros2 run scout2map_bridge pico_bridge --ros-args \
  -p port:=/dev/ttyACM0 \
  -p publish_raw_json:=true
```

현재 적용된 값은 실행 중에 조회할 수 있다.

```bash
ros2 param list /pico_bridge
ros2 param get /pico_bridge stale_air_quality_s
```

설치와 빌드 절차는 [레포 루트 README](../README.md)에 있다.

---

## 4. QoS

| 토픽 | 신뢰성 | 내구성 | depth |
|---|---|---|---|
| `/sensors/*` | RELIABLE | VOLATILE | 10 |
| `/bridge/status` | RELIABLE | TRANSIENT_LOCAL | 1 |

센서 데이터에는 BEST_EFFORT를 쓰는 관례가 있지만 여기서는 RELIABLE을 택했다.
데이터 레이트가 초당 수 건 수준이라 신뢰성 전송 비용이 사실상 없고,
**이벤트 엔진이 임계값 돌파 순간의 샘플을 조용히 놓치는 쪽이 훨씬 위험**하기 때문이다.

`/bridge/status`만 TRANSIENT_LOCAL이라 나중에 뜬 노드도 마지막 상태를 즉시 받는다.

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
ros2 topic echo /bridge/status --once
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
ros2 run scout2map_bridge pico_bridge --ros-args -p publish_raw_json:=true
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
  브릿지에도 송신 경로를 두지 않았다. 캘리브레이션 명령 등이 필요해지면
  `SerialReader`에 write 경로를 추가하고 펌웨어에 파서를 넣어야 한다.
- **`SerialReader`가 이 파일 안에 있다.** 포트 개방·재연결 로직 자체는 범용이지만
  아직 공용 모듈로 분리하지 않았다. STM32 브릿지는 프레이밍 방식이 다르므로
  (개행 분할이 아니라 SOF 탐색 + CRC16 검증) 그때 함께 정리한다.
- **`stamp`은 수신 시각이다.** MCU 측정 시각이 아니다. 자세한 내용은 msg 문서 2절 참조.
