# Mixer_Comm — demo branch

This branch adds two ROS 2 application layers on top of the bridge that lives
on `main`:

- **`mixer_demo_*`** — statistical bidirectional traffic with per-peer
  loss / duplicate / RTT accounting, optional bounded runs with JSON
  reports. Used to verify RF link quality numerically.
- **`mixer_hello_*`** — human-readable `"Hello, Mixer <id>-<counter>"`
  ASCII traffic. Used to eyeball that the bidirectional link is alive.

For everything else — prerequisites, firmware build/flash, the bridge
itself, the host↔dongle wire protocol, the Docker image, scaling to >2
nodes — see [`main`'s README](https://github.com/Porofly/Mixer_Comm/blob/main/README.md).

---

## What this branch adds on top of main

Three new executables in the `mixer_comm` package:

| Executable | Topic in/out | Purpose |
|------------|--------------|---------|
| `mixer_demo_pub` | sub `mixer/rx`, pub `mixer/tx` | originate sequenced + timestamped frames, echo any peer frame back |
| `mixer_demo_sub` | sub `mixer/rx` | per-peer rx / lost / dup / reorder + RTT histogram + final JSON report |
| `mixer_hello_pub` / `mixer_hello_sub` | same topics | ASCII hello-world counterpart of the demo pair |

Two new launch files:

- `mixer_demo_single.launch.py` — one dongle: bridge + demo_pub + demo_sub
- `mixer_hello.launch.py` — one dongle: bridge + hello_pub + hello_sub

`docker/run_jetson.sh` from `main` is extended to select between them via
`MIXER_MODE=demo` (default) or `MIXER_MODE=hello`.

---

## Two-Jetson verification (statistical demo)

Run one container per Jetson, each with its own dongle. ROS 2 DDS is
intentionally local-only on each board — all peer traffic crosses the
Mixer RF link, decoded locally.

The demo pub emits 16-byte frames at the firmware's round rate
(auto-detected via `mixer/stats`); when it sees a peer's frame on its own
`mixer/rx`, it mirrors `(sender_id, seq, origin_ts)` back in its next
outgoing frame. The original sender then closes the loop with
`RTT = now - echo_origin_ts`. Because `origin_ts` is read from the same
`steady_clock` used to compute the difference, **no inter-host clock
synchronisation is required** — each node measures RTT against its own
clock.

### Build (once, on each Jetson — see main README for prerequisites)

```bash
git clone --recursive <repo> && cd Mixer_Comm
git checkout demo
docker build --network=host -f docker/Dockerfile -t mixer_comm:latest .
```

`--network=host` works around an `iptable_raw`-missing kernel module on
Jetson L4T — see main README's Docker section.

### Run — unbounded (Ctrl-C to stop)

```bash
# Jetson A, dongle flashed as node 1 (find serial with `ls /dev/serial/by-id/`)
MIXER_HOST_NET=1 ./docker/run_jetson.sh 1 <SERIAL_NODE1>

# Jetson B, dongle flashed as node 2
MIXER_HOST_NET=1 ./docker/run_jetson.sh 2 <SERIAL_NODE2>
```

### Run — bounded with JSON report

```bash
# Both Jetsons (same MIXER_COUNT / MIXER_TIMEOUT_S):
MIXER_HOST_NET=1 MIXER_COUNT=100 MIXER_TIMEOUT_S=200 \
    ./docker/run_jetson.sh 2 <SERIAL_NODE2>
```

`MIXER_COUNT=N` means: pub originates N frames then switches to echo-only
keep-alive; sub waits for `received >= N` per peer + a 3 s drain window
to fold in late echoes, prints a final cumulative report, writes JSON to
`./reports/node<id>.json` on the host, and tears the launch down.
`MIXER_TIMEOUT_S=T` is a safety hard deadline in case RF losses keep
`received` short of N — strongly recommended for any bounded run.

### Reading the output

Every `report_period_s` (default 2 s) each `mixer_demo_sub` prints a line
per peer:

```
peer=1 rx=88 lost=0 dup=0 reord=0 rtt[n=42 min/mean/max us]=3707/4108/5651
```

- `rx` increasing on both sides → bidirectional RF link OK
- `lost` near 0 → link healthy *and* host publish rate is below the round
  rate (otherwise these are backpressure drops, not RF losses)
- `rtt n=` non-zero → the peer is correctly echoing our frames back, so
  the full path (host A → dongle A → RF → dongle B → host B → dongle B →
  RF → dongle A → host A) is intact
- `dup` rises after a bounded run hits its `tx_count` limit — those are
  the keep-alive frames, expected

The final JSON looks like:

```json
{
  "node_id": 2,
  "duration_s": 129.7,
  "expected_rx_count": 100,
  "peers": {
    "1": { "received": 102, "lost": 0, "duplicate": 6, "reordered": 0,
           "rtt_count": 97, "rtt_min_us": 3708, "rtt_mean_us": 4341,
           "rtt_max_us": 7890 }
  }
}
```

---

## Two-Jetson verification (Hello-world)

Same Jetson layout, `MIXER_MODE=hello`:

```bash
# Jetson A
MIXER_HOST_NET=1 MIXER_MODE=hello ./docker/run_jetson.sh 1 <SERIAL_NODE1>

# Jetson B
MIXER_HOST_NET=1 MIXER_MODE=hello ./docker/run_jetson.sh 2 <SERIAL_NODE2>
```

Each side prints both its own outgoing and the peer's incoming traffic:

```
hello_pub: rate=0.864 Hz (auto-sync)
tx: Hello, Mixer 2-0
rx (slot 0): "Hello, Mixer 1-0"
tx: Hello, Mixer 2-1
rx (slot 0): "Hello, Mixer 1-1"
```

`MIXER_COUNT` / `MIXER_TIMEOUT_S` have no effect in this mode — the
launch runs until Ctrl-C.

---

## Auto-sync rate

Both demo and hello publishers default to `pub_rate_hz=0.0`, which means:

1. Subscribe to local `mixer/stats` (one message per Mixer round).
2. Collect 4 samples (~3–4 s) of `(steady_clock, round_counter)`.
3. Compute round rate = Δround / Δtime.
4. Start the tx timer at 95 % of that rate.

Why 95 %: the host must publish slower than the round, otherwise extra
frames pile up in the dongle's per-round TX queue and look like loss.

The auto-sync makes the publishers self-tune to whatever round period the
firmware ended up with (`build_node.sh --round-length` etc., see main
README). Override with `pub_rate_hz:=1.0` (passed through `run_jetson.sh`
as a trailing arg) to skip auto-sync entirely.

---

## Launch arguments cheatsheet

`mixer_demo_single.launch.py`:

| Arg | Default | Notes |
|-----|---------|-------|
| `node_id` | (required) | Mixer node id this dongle was flashed for |
| `serial` | (required) | USB serial of the dongle |
| `slot` | `node_id - 1` | Mixer slot to publish into |
| `pub_rate_hz` | `0.0` | 0 = auto-sync, >0 = manual |
| `report_period_s` | `2.0` | sub's periodic print interval |
| `count` | `0` | 0 = unlimited, N>0 = bounded run |
| `drain_grace_s` | `3.0` | extra time after `count` to fold in late echoes |
| `timeout_s` | `0.0` | hard deadline; 0 = disabled |
| `report_path` | `""` | JSON output path; `""` = disabled |

`mixer_hello.launch.py`: only `node_id`, `serial`, `slot`, `pub_rate_hz`,
`baud`.

`run_jetson.sh` env-var pass-through:

| Env var | Effect |
|---------|--------|
| `MIXER_MODE` | `demo` (default) or `hello` |
| `MIXER_COUNT` | demo bounded mode (sets `count` and `report_path`) |
| `MIXER_TIMEOUT_S` | demo hard deadline |
| `MIXER_HOST_NET` | `1` to use `--network=host` (Jetson L4T workaround) |
| `MIXER_REPORTS_DIR` | host dir mounted at `/tmp/mixer_reports` (default `./reports`) |
| `MIXER_IMAGE`, `MIXER_NAME` | image tag and container name overrides |

Trailing positional args after `<node_id> <serial>` are forwarded to
`ros2 launch`, e.g. `pub_rate_hz:=1.0`.
