#!/usr/bin/env bash
# One-shot helper to run the mixer_comm container on a Jetson with a single
# Mixer dongle attached. Brings up:
#   - mixer_serial_node  (USB <-> ROS 2 bridge for this dongle)
#   - mixer_demo_pub     (1 Hz synthetic 16-byte frames + RTT echo)
#   - mixer_demo_sub     (per-peer rx/lost/dup/reorder + RTT report)
#
# Usage:
#   ./docker/run_jetson.sh <node_id> <usb_serial> [extra ros2 launch args...]
#
# Example (Jetson A, dongle flashed as node 1):
#   ./docker/run_jetson.sh 1 297729DAE31AEE29
# (Jetson B, dongle flashed as node 2):
#   ./docker/run_jetson.sh 2 5B36F76056801B1F
#
# Notes:
# - Run from the repo root: ./docker/run_jetson.sh ...
# - Defaults to local-only DDS (ROS_LOCALHOST_ONLY=1, baked into the image)
#   so the two Jetsons do NOT discover each other over the network. All peer
#   traffic is carried by the Mixer RF link, decoded locally by the dongle.
# - Mounts /dev/serial and /dev/bus/usb so the by-id symlink the launch file
#   expects resolves inside the container.
# - Set MIXER_HOST_NET=1 to use --network=host. Needed on Jetson L4T kernels
#   that lack the iptable_raw module (modprobe iptable_raw -> FATAL), which
#   makes Docker's bridge driver fail. Safe with ROS_LOCALHOST_ONLY=1: DDS
#   stays on lo even on the host network namespace.
# - Set MIXER_COUNT=N for a bounded run: pub stops originating after N frames
#   (it then keeps emitting echo-only keep-alives so the peer's RTT closes
#   out), sub waits for N received frames per peer + drain grace, then
#   writes a JSON summary to ./reports/node<id>.json on the host (mounted
#   into the container at /tmp/mixer_reports). With MIXER_COUNT unset
#   (default), runs forever.
# - Set MIXER_TIMEOUT_S=120 to bound the run by wall time as well: if RF
#   losses keep received < N, the sub finalizes anyway after this many
#   seconds. Strongly recommended for any MIXER_COUNT run.

set -euo pipefail

if [[ $# -lt 2 ]]; then
    echo "usage: $0 <node_id> <usb_serial> [extra ros2 launch args...]" >&2
    exit 2
fi

NODE_ID=$1
SERIAL=$2
shift 2

IMAGE=${MIXER_IMAGE:-mixer_comm:latest}
NAME=${MIXER_NAME:-mixer-node${NODE_ID}}

# Sanity check: the by-id symlink for this dongle exists on the host, and
# resolve it to the actual /dev/ttyACM* node so we can pass that node into
# the container as a --device. Mounting /dev/serial alone keeps the by-id
# symlink visible but the symlink target (/dev/ttyACM*) must also be exposed
# explicitly; --device-cgroup-rule grants permission but does not create the
# device node inside the container.
BY_ID="/dev/serial/by-id/usb-NES_Lab___Mixer_Mixer_Console__PCA10059__${SERIAL}-if00"
if [[ ! -e "$BY_ID" ]]; then
    echo "ERROR: ${BY_ID} not found on host." >&2
    echo "  - Is the dongle plugged in?" >&2
    echo "  - Is its USB serial really ${SERIAL}? (ls /dev/serial/by-id/)" >&2
    exit 1
fi
TTY_DEV=$(readlink -f "$BY_ID")
if [[ ! -c "$TTY_DEV" ]]; then
    echo "ERROR: by-id symlink resolved to '$TTY_DEV' which is not a char device." >&2
    exit 1
fi

NET_ARGS=()
if [[ "${MIXER_HOST_NET:-0}" == "1" ]]; then
    NET_ARGS+=(--network=host)
fi

# Reports directory: created on the host (./reports by default) and mounted
# at /tmp/mixer_reports inside the container so JSON summaries land where the
# user can see them after the container exits.
REPORTS_DIR_HOST=${MIXER_REPORTS_DIR:-$(pwd)/reports}
mkdir -p "$REPORTS_DIR_HOST"

LAUNCH_ARGS=(node_id:=${NODE_ID} serial:=${SERIAL})
if [[ -n "${MIXER_COUNT:-}" && "${MIXER_COUNT}" != "0" ]]; then
    LAUNCH_ARGS+=(
        count:=${MIXER_COUNT}
        report_path:=/tmp/mixer_reports/node${NODE_ID}.json
    )
fi
if [[ -n "${MIXER_TIMEOUT_S:-}" && "${MIXER_TIMEOUT_S}" != "0" ]]; then
    LAUNCH_ARGS+=(timeout_s:=${MIXER_TIMEOUT_S})
fi

exec docker run --rm -it \
    --name "$NAME" \
    "${NET_ARGS[@]}" \
    -v /dev/serial:/dev/serial \
    -v /dev/bus/usb:/dev/bus/usb \
    -v "${REPORTS_DIR_HOST}:/tmp/mixer_reports" \
    --device "${TTY_DEV}:${TTY_DEV}" \
    "$IMAGE" \
    ros2 launch mixer_comm mixer_demo_single.launch.py \
        "${LAUNCH_ARGS[@]}" "$@"
