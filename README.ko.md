# Mixer_Comm

> English: see [README.md](README.md).

**Mixer** 무선 many-to-all 통신 프로토콜을 nRF52840 USB 동글(PCA10059) 위에서
구동하고, 그 위에 ROS 2 브리지를 얹은 레포지토리입니다. 구성:

- `Mixer/` — 업스트림 Mixer 펌웨어 fork (git 서브모듈). USB CDC를 통해 호스트와
  이진 프로토콜로 통신하도록 수정되어 있습니다.
- `ros2_ws/` — colcon 워크스페이스, 두 개의 ROS 2 패키지 포함:
  - `mixer_comm_msgs` — 메시지 정의 (`MixerStats`, `MixerPayload`)
  - `mixer_comm` — `mixer_serial_node` 와 launch 파일. `dongles.yaml`로 `node_id`
    ↔ USB 시리얼 번호 매핑을 관리하면서 동글 하나당 노드 하나를 띄웁니다.

최종 상태: 각 동글이 ROS 2 namespace 하나가 되고, 동글 사이에 임의의 16바이트
페이로드를 평범한 ROS 2 토픽으로 주고받습니다.

```
+-------------+   /mixer/nodeN/mixer/tx        +-----------+
| ROS 2 app   | -----------------------------> |           |
|             |   /mixer/nodeN/mixer/rx        | dongle N  |  RF mesh  ...
|             | <----------------------------- |           |
+-------------+   /mixer/nodeN/mixer/stats     +-----------+
                  /mixer/nodeN/mixer/log
```

---

## 사전 준비

| 도구 | 검증된 버전 | 용도 |
|------|-------------|------|
| Ubuntu / ROS 2 Jazzy | 24.04 / Jazzy | 호스트 런타임 |
| colcon, rosdep | ROS 2 기본 포함 | 호스트 빌드 |
| SEGGER Embedded Studio for ARM | **5.70a** (5.x 계열 필수) | 펌웨어 빌드 |
| `nrfutil` | 8.x | DFU 플래시 |

SES 설치, `nrfutil` 셋업, dialout 그룹 권한 등의 자세한 절차는
[Mixer/tutorial/nRF52840/PCA10059.md](Mixer/tutorial/nRF52840/PCA10059.md)
를 참고하세요.

서브모듈 포함 클론:

```bash
git clone --recursive <repo>
# 또는 일반 클론 후:
git submodule update --init --recursive
```

---

## 빌드

### 펌웨어 (노드 ID별)

```bash
cd Mixer
scripts/build_node.sh 1   # tutorial/nRF52840/build/node1/mixer_node1.zip 생성
scripts/build_node.sh 2
```

스크립트는 `BUILD_NODE_ID`를 다시 주입하고 SES(`emBuild`)로 빌드합니다.
SES 설치 경로가 기본값(`/usr/share/segger_embedded_studio_for_arm_5.70a/`)이
아니라면 `SES_DIR=...` 환경변수로 알려주세요.

### 호스트 (ROS 2 워크스페이스)

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

---

## 플래시

각 동글의 **RESET 버튼을 짧게 한 번 눌러** DFU 모드로 진입시켜야 합니다.
진입 성공 시 빨간 LED가 빠르게 깜빡입니다.

```bash
cd Mixer
scripts/flash_node.sh 1               # 단일 DFU 동글 자동 감지
scripts/flash_node.sh 1 /dev/ttyACM2  # 여러 동글이 DFU 모드일 때는 명시
```

플래시 후 동글이 펌웨어 모드로 재부팅되며 `1915:000a Mixer Console (PCA10059)`
로 enumerate 됩니다. `lsusb`에서 안 보이면 USB에서 뽑았다 다시 꽂으세요
(PCA10059 stale enumeration 버그, `PCA10059.md` §2.1 참고).

---

## 실행

Launch 파일은 [`ros2_ws/src/mixer_comm/config/dongles.yaml`](ros2_ws/src/mixer_comm/config/dongles.yaml)
를 읽어 yaml 엔트리 하나당 `mixer_serial_node` 하나를 spawn 합니다.
namespace는 `/mixer/node<id>`, device 경로는 USB 시리얼 번호 기반의
`/dev/serial/by-id/...` 안정 경로를 사용합니다.

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch mixer_comm mixer_bridge.launch.py
```

다른 테스트벤치용 매핑을 쓰려면:

```bash
ros2 launch mixer_comm mixer_bridge.launch.py config:=/path/to/other.yaml
```

yaml에 적힌 동글이 실제로 안 꽂혀 있으면 launch가 즉시 실패하면서 어떤
by-id 경로/어떤 시리얼이 누락됐는지 출력합니다.

### 토픽 카탈로그

각 동글 (`node_id = N`) 별로:

| 토픽 | 방향 | 타입 | 의미 |
|------|------|------|------|
| `/mixer/nodeN/mixer/tx` | sub | `mixer_comm_msgs/MixerPayload` | 지정 슬롯에 페이로드를 큐잉. 해당 동글 소유 슬롯이 아니면 무시 |
| `/mixer/nodeN/mixer/rx` | pub | `mixer_comm_msgs/MixerPayload` | 라운드마다 디코드된 슬롯 페이로드 (슬롯 번호는 `slot` 필드) |
| `/mixer/nodeN/mixer/stats` | pub | `mixer_comm_msgs/MixerStats` | 라운드별 카운터 (rank, decoded, not_decoded, weak, wrong) |
| `/mixer/nodeN/mixer/log` | pub | `std_msgs/String` | 펌웨어가 보낸 텍스트 (부팅 배너, FATAL 등) |

### 빠른 동작 확인 (동글 2개)

```bash
# 셸 1
ros2 launch mixer_comm mixer_bridge.launch.py

# 셸 2 — 수신 측 모니터
ros2 topic echo /mixer/node2/mixer/rx

# 셸 3 — 송신 측 publish
ros2 topic pub --once /mixer/node1/mixer/tx mixer_comm_msgs/msg/MixerPayload \
    "{slot: 0, data: [222,173,190,239,17,17,17,17,34,34,34,34,51,51,51,51]}"
```

기대 결과: 다음 slot 0 메시지에 위 페이로드가 그대로 도착, 두 동글의
`stats` 토픽이 `rank=8 decoded=8 not_decoded=0`을 보고합니다.

---

## 호스트 ↔ 동글 와이어 프로토콜

USB CDC, 115200 8N1, 길이 prefix 이진 프레이밍. 정의는
[`Mixer/tutorial/nRF52840/host_proto.h`](Mixer/tutorial/nRF52840/host_proto.h)
에 있고, 호스트 측은
[`ros2_ws/src/mixer_comm/include/mixer_comm/frame_codec.hpp`](ros2_ws/src/mixer_comm/include/mixer_comm/frame_codec.hpp)
에서 mirror.

```
[size_lo][size_hi][type][slot][payload ...]
   1B       1B      1B    1B    size-2 bytes
```

`size`는 `type + slot + payload` 길이를 셉니다. 프레임 타입:

| 타입 | 방향 | 페이로드 |
|------|------|---------|
| `0x10` TX_PAYLOAD | host → dongle | 정확히 `MX_PAYLOAD_SIZE` 바이트 |
| `0x20` RX_PAYLOAD | dongle → host | 정확히 `MX_PAYLOAD_SIZE` 바이트 |
| `0x30` ROUND_STATS | dongle → host | 7 × little-endian uint32 (`hp_round_stats_t`) |
| `0x40` LOG | dongle → host | ASCII 텍스트, 내부 프레이밍 없음 |

부팅 직후 동글이 LOG 프레임으로 `"mixer-binary-proto v1\n"` 한 줄을 보냅니다.
그 이후 모든 출력은 이진. 호스트는 size 필드가 비현실적일 때 (`< 2` 또는
`> kMaxFrameSize`) 1바이트씩 버려가며 재동기화합니다.

---

## 노드 확장

검증된 2-동글 구성에서 노드 수를 늘리려면 두 파일만 손보면 됩니다.

1. **펌웨어** — [`Mixer/tutorial/nRF52840/mixer_config.h`](Mixer/tutorial/nRF52840/mixer_config.h):

   ```c
   // 4-노드 예시
   static const uint8_t nodes[]                = { 1, 2, 3, 4 };
   static const uint8_t payload_distribution[] = { 1, 2, 3, 4, 1, 2, 3, 4 };
   ```

   노드 ID마다 펌웨어를 다시 빌드해 플래시. `MX_GENERATION_SIZE`는
   `payload_distribution[]` 길이를 자동으로 따라가고, `MX_ROUND_LENGTH = 50`
   슬롯이면 16개 정도까지는 여유 있게 수용합니다.

2. **호스트** — [`ros2_ws/src/mixer_comm/config/dongles.yaml`](ros2_ws/src/mixer_comm/config/dongles.yaml):

   ```yaml
   dongles:
     - { node_id: 1, serial: "<동글 1 시리얼>" }
     - { node_id: 2, serial: "<동글 2 시리얼>" }
     - { node_id: 3, serial: "<동글 3 시리얼>" }
     - { node_id: 4, serial: "<동글 4 시리얼>" }
   ```

   각 동글의 시리얼은 꽂은 후 `ls /dev/serial/by-id/`로 확인합니다.

   코드 변경/재빌드 불필요.

2 노드 → N > 2 노드로 넘어갈 때는 동기화 재확인이 필요합니다. Mixer는 매
라운드 모든 슬롯이 송신해야 네트워크 동기화가 유지됩니다. 본 펌웨어는
호스트 큐가 비어 있을 때도 zero 페이로드를 채워 송신하므로
(`MX_WEAK_ZEROS=0`) idle 트래픽에서도 동기화가 살아 있습니다.

---

## 레이아웃

```
.
├── CLAUDE.md                 # AI 어시스트 작업 가이드
├── Mixer/                    # 서브모듈 (Porofly/Mixer fork)
│   ├── src/
│   ├── tutorial/nRF52840/    # PCA10059 튜토리얼 + 이진 프로토콜 펌웨어
│   └── scripts/{build,flash}_node.sh
└── ros2_ws/
    └── src/
        ├── mixer_comm_msgs/
        │   └── msg/{MixerStats,MixerPayload}.msg
        └── mixer_comm/
            ├── include/mixer_comm/{frame_codec,serial_port}.hpp
            ├── src/{frame_codec,serial_port,mixer_serial_node}.cpp
            ├── config/dongles.yaml
            └── launch/mixer_bridge.launch.py
```
