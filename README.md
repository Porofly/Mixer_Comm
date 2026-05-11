# Mixer_Comm

> 한국어 문서는 [README.ko.md](README.ko.md) 를 참고하세요.

ROS 2 bridge for the **Mixer** wireless many-to-all communication protocol,
running on nRF52840 USB dongles (PCA10059). The repository contains:

- `Mixer/` — git submodule with a fork of the upstream Mixer firmware,
  modified to speak a binary protocol with the host over USB CDC.
- `ros2_ws/` — colcon workspace with two ROS 2 packages:
  - `mixer_comm_msgs` — message definitions (`MixerStats`, `MixerPayload`).
  - `mixer_comm` — `mixer_serial_node` and a launch file that brings up one
    node per dongle, mapping `node_id` ↔ USB serial via `dongles.yaml`.

End state: each dongle becomes a ROS 2 namespace, and arbitrary 16-byte
payloads can be exchanged between dongles over Mixer rounds via plain
ROS 2 topics.

```
+-------------+   /mixer/nodeN/mixer/tx        +-----------+
| ROS 2 app   | -----------------------------> |           |
|             |   /mixer/nodeN/mixer/rx        | dongle N  |  RF mesh  ...
|             | <----------------------------- |           |
+-------------+   /mixer/nodeN/mixer/stats     +-----------+
                  /mixer/nodeN/mixer/log
```

---

## Prerequisites

| Tool | Version tested | Purpose |
|------|----------------|---------|
| Ubuntu / ROS 2 Jazzy | 24.04 / Jazzy | host runtime |
| colcon, rosdep | shipped with ROS 2 | host build |
| SEGGER Embedded Studio for ARM | **5.70a** (5.x required) | firmware build |
| `nrfutil` | 8.x | DFU flashing |

See [Mixer/tutorial/nRF52840/PCA10059.md](Mixer/tutorial/nRF52840/PCA10059.md)
for SES installation, `nrfutil` setup, and dialout group permissions.

Clone with submodules:

```bash
git clone --recursive <repo>
# or, after a plain clone:
git submodule update --init --recursive
```

---

## Build

### Firmware (per node ID)

```bash
cd Mixer
scripts/build_node.sh 1   # produces tutorial/nRF52840/build/node1/mixer_node1.zip
scripts/build_node.sh 2
```

Each invocation re-injects `BUILD_NODE_ID` and rebuilds with SES (`emBuild`).
Pass `SES_DIR=...` if your SES install is not at the default
`/usr/share/segger_embedded_studio_for_arm_5.70a/`.

### Host (ROS 2 workspace)

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

---

## Flash

Each dongle must be put into DFU mode by **pressing the RESET button briefly**.
The red LED then blinks fast.

```bash
cd Mixer
scripts/flash_node.sh 1               # auto-detects a single DFU dongle
scripts/flash_node.sh 1 /dev/ttyACM2  # explicit when several are in DFU mode
```

After flashing, the dongle reboots into firmware mode and enumerates as
`1915:000a Mixer Console (PCA10059)`. If `lsusb` does not show it, unplug and
re-insert (PCA10059 stale-enumeration quirk; see `PCA10059.md` §2.1).

---

## Run

The launch file reads [`ros2_ws/src/mixer_comm/config/dongles.yaml`](ros2_ws/src/mixer_comm/config/dongles.yaml)
and spawns one `mixer_serial_node` per entry under namespace `/mixer/node<id>`,
using the dongle's USB serial number for a stable `/dev/serial/by-id/...` path.

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch mixer_comm mixer_bridge.launch.py
```

Override the config path for an alternate testbed:

```bash
ros2 launch mixer_comm mixer_bridge.launch.py config:=/path/to/other.yaml
```

If a dongle in the yaml is missing at runtime, launch fails fast with the
expected by-id path and the missing serial.

### Topic catalogue

For each dongle with `node_id = N`:

| Topic | Direction | Type | Meaning |
|-------|-----------|------|---------|
| `/mixer/nodeN/mixer/tx` | sub | `mixer_comm_msgs/MixerPayload` | enqueue a payload for the named slot; ignored if the slot is not owned by this dongle |
| `/mixer/nodeN/mixer/rx` | pub | `mixer_comm_msgs/MixerPayload` | one message per slot decoded each round; `slot` identifies which slot the bytes came from |
| `/mixer/nodeN/mixer/stats` | pub | `mixer_comm_msgs/MixerStats` | per-round counters (rank, decoded, not_decoded, weak, wrong) |
| `/mixer/nodeN/mixer/log` | pub | `std_msgs/String` | text lines emitted by the firmware (boot banner, FATAL) |

### Quick smoke test (two dongles)

```bash
# in one shell:
ros2 launch mixer_comm mixer_bridge.launch.py

# in another:
ros2 topic echo /mixer/node2/mixer/rx   # watch the receive side

# in a third:
ros2 topic pub --once /mixer/node1/mixer/tx mixer_comm_msgs/msg/MixerPayload \
    "{slot: 0, data: [222,173,190,239,17,17,17,17,34,34,34,34,51,51,51,51]}"
```

Expected: the next slot-0 message on `/mixer/node2/mixer/rx` carries that
exact 16-byte payload. Both dongles' `stats` topics should report
`rank=8 decoded=8 not_decoded=0`.

---

## Host ↔ dongle wire protocol

USB CDC, 115200 8N1, length-prefixed binary frames. Definition lives in
[`Mixer/tutorial/nRF52840/host_proto.h`](Mixer/tutorial/nRF52840/host_proto.h)
and is mirrored on the host side in
[`ros2_ws/src/mixer_comm/include/mixer_comm/frame_codec.hpp`](ros2_ws/src/mixer_comm/include/mixer_comm/frame_codec.hpp).

```
[size_lo][size_hi][type][slot][payload ...]
   1B       1B      1B    1B    size-2 bytes
```

`size` counts `type + slot + payload`. Frame types:

| Type | Direction | Payload |
|------|-----------|---------|
| `0x10` TX_PAYLOAD | host → dongle | exactly `MX_PAYLOAD_SIZE` bytes |
| `0x20` RX_PAYLOAD | dongle → host | exactly `MX_PAYLOAD_SIZE` bytes |
| `0x30` ROUND_STATS | dongle → host | 7 × little-endian uint32 (`hp_round_stats_t`) |
| `0x40` LOG | dongle → host | ASCII text, no inner framing |

On boot the dongle emits one LOG frame with `"mixer-binary-proto v1\n"`. After
that, all output is binary. The host side resyncs by discarding bytes whenever
the size field looks implausible (`< 2` or `> kMaxFrameSize`).

---

## Extending to more nodes

The two-dongle setup verified above generalises by editing two files:

1. **Firmware** — [`Mixer/tutorial/nRF52840/mixer_config.h`](Mixer/tutorial/nRF52840/mixer_config.h):

   ```c
   // 4-node example
   static const uint8_t nodes[]                = { 1, 2, 3, 4 };
   static const uint8_t payload_distribution[] = { 1, 2, 3, 4, 1, 2, 3, 4 };
   ```

   Rebuild and flash one image per node ID. `MX_GENERATION_SIZE` follows
   `payload_distribution[]` automatically; `MX_ROUND_LENGTH = 50` slots
   leaves comfortable headroom up to ~16 distinct slots.

2. **Host** — [`ros2_ws/src/mixer_comm/config/dongles.yaml`](ros2_ws/src/mixer_comm/config/dongles.yaml):

   ```yaml
   dongles:
     - { node_id: 1, serial: "297729DAE31AEE29" }
     - { node_id: 2, serial: "5B36F76056801B1F" }
     - { node_id: 3, serial: "<new serial>" }
     - { node_id: 4, serial: "<new serial>" }
   ```

   No code or rebuild needed.

When jumping from 2 to N>2 nodes, expect to re-check synchronisation: Mixer
relies on every slot transmitting each round so the network stays locked.
This firmware fills empty TX slots with a zero payload on purpose (with
`MX_WEAK_ZEROS=0`) so synchronisation survives idle traffic.

---

## Layout

```
.
├── CLAUDE.md                 # working-agreement notes for AI-assisted edits
├── Mixer/                    # submodule (Porofly/Mixer fork)
│   ├── src/
│   ├── tutorial/nRF52840/    # PCA10059 tutorial + binary-protocol firmware
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
