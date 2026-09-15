# LSLIDAR C16 on ROS 2 Humble — bring-up, root causes and fix

Working notes for the **LSLIDAR C16-151A-20110501** unit on this bench, driving the vendor
`Lslidar_ROS2_driver` (CX ROS2 driver **4.2.4**) under **ROS 2 Humble** / Ubuntu 22.04.

Out of the box the driver produced **no point cloud at all**. Three independent problems were
involved. This document records each, the evidence, and the fix that is now in place.

---

## TL;DR

| # | Problem | Fix |
|---|---------|-----|
| 1 | LiDAR sends DIFOP to port **2368**, the same port as MSOP. Driver listens on 2369, never receives DIFOP, and therefore discards **every** MSOP packet. | Driver patched to demultiplex both streams off one socket when `difop_port == msop_port`. Set `difop_port: 2368`. |
| 2 | Model auto-detection reads a byte past the end of a 1206-byte packet and misidentifies the C16 as a **C32**, producing geometrically wrong clouds. | New `lidar_type_override` parameter. Set `lidar_type_override: "c16_3"`. |
| 3 | Host address `192.168.1.102` kept vanishing, silently killing all traffic. | Duplicate NetworkManager profiles. Address moved into the profile itself. |

Current state: point cloud publishes at **10 Hz**, **16 rings**, ~2° spacing, ±15° vertical.

---

## Hardware and network facts

Decoded from the LiDAR's own DIFOP packet (header `a5 ff 00 5a`):

```
lidar IP   : 192.168.1.200
dest IP    : 192.168.1.102
MAC        : 50:3e:7c:00:02:1f
msop port  : 2368
difop port : 2368          <-- the anomaly; factory default is 2369
motor rpm  : 600           (-> 10 Hz)
```

- MSOP packets are **1206 bytes** (the v3.0 generation format, not the 1212-byte v4.0 one).
- `data[1204] = 0x39` -> **dual return** mode.
- Host NIC is the USB ethernet adapter `enxa0cec897912a`, which must hold `192.168.1.102/24`.

---

## Problem 1 — DIFOP arrives on the MSOP port

### Symptom

```
[lslidar_driver_node-1] Opening UDP socket msop port: 2368
[lslidar_driver_node-1] Opening UDP socket difop port: 2369
[lslidar_driver_node-1] 2026-9-10 13:54:38 lslidar poll() timeout, port: 2369
[lslidar_driver_node-1] 2026-9-10 13:54:41 lslidar poll() timeout, port: 2369
```

The topic `/cx/lslidar_point_cloud` is advertised but **never publishes**. `ros2 topic hz` reports
nothing. rviz shows an empty scene. The only message is a `poll() timeout` on 2369, which reads
like a harmless warning.

### Evidence

Binding a raw UDP socket to each port showed data on 2368 only:

```
port 2368: 10035 packets in 6s, src=('192.168.1.200', 2368), sizes={1206}
port 2369: 0 packets in 6s
```

Scanning ports 2340-2420 plus common alternatives found DIFOP nowhere. Classifying the 2368
stream *by packet header* found both kinds mixed together:

```
total on 2368: 8363
headers seen: {'ff ee ..': 8348, 'a5 ff 00 5a': 15}      # ~3 DIFOP/s
```

`ff ee` = MSOP, `a5 ff 00 5a` = DIFOP. The LiDAR's stored config confirmed it: `difop port: 2368`.

### Why it kills the point cloud

Not merely cosmetic. In stock CX 4.2.4 (line numbers refer to the **unpatched** source, preserved in `.backup-20260910-144353/`):

- `determineLidarType()` is called **only from inside `difopPoll()`** — `src/lslidar_driver.cpp:392`.
- `determineLidarType()` is the only thing that sets `start_process_msop_ = true` (line 1658).
- `poll()` line 555: `if (!checkPacketValidity(packet) || !start_process_msop_) return false;`

No DIFOP -> no lidar type -> `start_process_msop_` stays false -> all ~1670 MSOP packets/sec are
received and silently thrown away.

### Why the obvious fix does not work

Simply setting `difop_port: 2368` on the stock driver is not enough: it creates **two UDP sockets
on the same port**. With `SO_REUSEADDR`, Linux delivers each unicast datagram to only *one* socket,
so one of the two threads starves. Worse, `difopPoll()` does `return` (killing its own thread) on
any packet whose header is not DIFOP.

Hence the demultiplexing patch below.

---

## Problem 2 — the C16 is misdetected as a C32

Once DIFOP was delivered (verified with a temporary relay), the driver ran but printed:

```
lidar type: C32, version 3.0
return mode: 2
total working time: -12351539 hours: -57 minutes.     <-- nonsense, same root cause
```

It published at 10 Hz, but with the **wrong geometry**:

| | rings | spacing | vertical span |
|---|---|---|---|
| Misdetected as C32 | 32 | ~0.9° | −12.1° … +13.8° |
| Correct for C16 | 16 | 2° | −15° … +15° |

### Cause

`determineLidarType()` switches on `pkt->data[1211]`. The message buffer is `uint8[1212]`, but this
LiDAR's MSOP packets are only **1206 bytes**, so `data[1206..1211]` is always zero padding.
`data[1211] == 0x00` therefore always matches, and with `data[1205] == 0x20` execution lands in the
`c32_3` branch:

```cpp
} else if (pkt->data[1211] == 0x00 && pkt->data[1205] == 0x20) {   // -> "C32, version 3.0"
```

`data[1205] == 0x20` was constant across ~8300 sampled packets, so this is deterministic, not a
fluke. There is no reliable model byte in this packet format, so byte-sniffing cannot be repaired —
the model must be stated explicitly.

### Correct profile for this unit — `c16_3`

```
vertical angles : c16_30_vertical_angle = {-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15}
ring_           : 16
lidar_number_   : 16
R1              : R2_ (0.0431)
conversionAngle : conversionAngle_C16_3 (1468)
distance_unit   : 0.25
```

Consistent with the 1206-byte packet format and with the standalone C16 driver's defaults
(`distance_unit: 0.25`, `degree_mode: 2`, `rpm: 600`).

---

## Problem 3 — host IP wiped by duplicate NetworkManager profiles

Twice during debugging the LiDAR "went dead": ping failed and every socket went silent, while the
NIC counter still climbed at ~1680 pkt/s (packets arriving, kernel dropping them because no local
address matched).

Cause: the adapter is NetworkManager-managed and there were **six** profiles all named
`camera-static`, five bound to `enxa0cec897912a`, all `autoconnect=yes` at priority 0:

| UUID prefix | address |
|---|---|
| `89aa6ccc` | 192.168.0.100/24 — the one actually used |
| `7cbf2f58` | **192.168.1.200/24 — the LiDAR's own IP (conflict)** |
| `aaf9155e` | 192.168.0.100/24 |
| `139f053f` | 192.168.0.100/24 |
| `c1830742` | 192.168.0.10/24 |

None carried `192.168.1.102`. Every time the USB adapter re-enumerated, NM reactivated a profile
and wiped any manually added address:

```
NetworkManager[808]: device (enxa0cec897912a): state change: activated -> unmanaged (reason 'removed')
NetworkManager[808]: device (eth0): interface index 75 renamed iface from 'eth0' to 'enxa0cec897912a'
NetworkManager[808]: device (enxa0cec897912a): Activation: starting connection 'camera-static'
```

`ip addr add` is therefore only a temporary patch — the address has to live in the profile.

---

## The fix

### A. Network (once, persistent)

```bash
sudo nmcli connection modify 89aa6ccc-69b3-4d2d-acae-9fd702d73487 +ipv4.addresses 192.168.1.102/24
sudo nmcli connection up     89aa6ccc-69b3-4d2d-acae-9fd702d73487
```

Recommended cleanup of the strays, especially the one holding the LiDAR's own address:

```bash
sudo nmcli connection delete 7cbf2f58-9be4-488d-8e62-be1535113d4a   # 192.168.1.200 — conflicts
sudo nmcli connection delete aaf9155e-4977-472a-926c-f84a2aa4ebed \
                             139f053f-85ee-4815-a24c-9e8b20d80dd7 \
                             c1830742-dc12-42d2-8f29-d07a7c8ef98e
```

Verify: `ip -br addr show enxa0cec897912a` must list `192.168.1.102/24`.

### B. Driver patch

Vendor sources were modified in place. Full diff: `lslidar_c16_local_fix.patch`.
Backup of the originals: `src/Lslidar_ROS2_driver/lslidar_driver/.backup-20260910-144353/`.

Both features are **inert unless explicitly enabled**, so stock behaviour is preserved for other
LiDAR models.

**1. DIFOP/MSOP demultiplexing** — active only when `difop_port == msop_port`:

- `createRosIO()` skips creating the second socket.
- `poll()` inspects each packet; `a5 ff 00 5a` frames are pushed onto `difop_queue_` and
  `poll()` returns early instead of treating them as MSOP.
- New `getDifopPacket()` feeds `difopPoll()` from that queue (guarded by a mutex + condition
  variable, 1 s timeout returning `1` to mirror `InputSocket`'s timeout semantics), so the large
  existing DIFOP parsing body is untouched.
- `poll()` also records `data[1204]` into `msop_return_byte_` for return-mode detection.

**2. `lidar_type_override` parameter** — accepts `c16_3`, `C16`, `c32_3`, `C32`; empty string keeps
the stock auto-detection. New `applyLidarTypeOverride()` replaces the `determineLidarType()` call
at the DIFOP call site, so factory calibration data from DIFOP is still parsed first.

### C. Configuration

`src/Lslidar_ROS2_driver/lslidar_driver/params/lslidar_cx.yaml`:

```yaml
    msop_port: 2368
    difop_port: 2368               # this unit sends DIFOP to the MSOP port
    lidar_type_override: "c16_3"   # c16_3 / C16 / c32_3 / C32  ("" = auto-detect)
```

### Build

```bash
cd ~/Documents/lslidar_c16_ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select lslidar_driver --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## Verification

```bash
ros2 launch lslidar_driver lslidar_cx_rviz_launch.py
```

Expected log — note the demux line and the `(override)` tag:

```
Opening UDP socket msop port: 2368
Opening UDP socket difop port: 2368
DIFOP shares the MSOP port 2368; demultiplexing by packet header
lidar type (override): C16, version 3.0
return mode: 2
```

Checks:

```bash
ros2 topic hz /cx/lslidar_point_cloud     # -> ~10 Hz
```

Measured ring geometry after the fix — 16 rings, ~1.9° spacing, top ring +14.3°:

```
ring : median elevation(deg)
   4 :   -6.66      10 :    4.83
   5 :   -4.79      11 :    6.64
   6 :   -2.87      12 :    8.51
   7 :   -0.96      13 :   10.43
   8 :    0.93      14 :   12.30
   9 :    2.85      15 :   14.29
```

(Lowest rings show no returns in an indoor scene — they point down at the floor inside `min_range`.)

In rviz: Fixed Frame `laser_link`, add a **PointCloud2** on `/cx/lslidar_point_cloud`.

---

## Known cosmetic issue

The working-time block still prints nonsense:

```
total working time: -12351539 hours: -57 minutes.
```

The unit takes the `fpga_type = 4` DIFOP branch (`data[1198] = 0x14`, and `0x14 & 0x0F == 4`), which
parses working-time counters at offsets this firmware does not populate. Harmless — it does not
affect the point cloud. Left alone deliberately rather than papering over vendor code.

Also note `config_vertical_angle_32[]` is only filled in the `fpga_type == 3` branch, so this unit
falls back to the nominal `c16_30_vertical_angle` table rather than per-unit factory angles.

---

## Alternative considered: a different driver

The official `Lslidar/Lslidar_ROS2_driver` has 34 per-model branches. **`C16_V3.0` and `C16_V4.0`
were downloaded and diffed — both are byte-identical to this tree** (same 1669-line
`lslidar_driver.cpp`, same "CX ROS2 driver version: 4.2.4"). Switching official branches achieves
nothing.

[`LS-Technical-Supporter/LS-LIDAR-C16ROS2`](https://github.com/LS-Technical-Supporter/LS-LIDAR-C16ROS2)
is a genuine alternative: older two-package design, hardcoded 16-beam `LSC16`, and **no DIFOP
gating** — DIFOP only *revises* angle corrections, so it decodes from MSOP alone. Its defaults match
this unit (`rpm: 600`, `distance_unit: 0.25`). Its README is empty, however, and patching the
current driver kept the richer feature set, so it was not adopted.

---

## Reverting

```bash
cd ~/Documents/lslidar_c16_ros2/src/Lslidar_ROS2_driver/lslidar_driver
cp .backup-20260910-144353/lslidar_driver.cpp src/
cp .backup-20260910-144353/lslidar_driver.h   include/lslidar_driver/
cp .backup-20260910-144353/lslidar_cx.yaml    params/
```

Then rebuild. Note the stock driver cannot work with this LiDAR unless the LiDAR's own DIFOP port
is reconfigured back to 2369.

---

## Appendix — diagnostic recipes

**Is the LiDAR sending, and to where?**

```bash
ip -br addr show enxa0cec897912a          # must include 192.168.1.102/24
ping -c3 192.168.1.200
cat /sys/class/net/enxa0cec897912a/statistics/rx_packets   # sample twice; ~1680/s when streaming
```

A climbing rx counter together with a failing ping means packets arrive but no local address
matches — i.e. problem 3.

**Which packet types arrive on which port** (no root needed, driver must be stopped):

```python
import socket, collections
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(("0.0.0.0", 2368)); s.settimeout(1)
c = collections.Counter()
for _ in range(5000):
    try: d, _ = s.recvfrom(2048)
    except socket.timeout: break
    c["MSOP" if d[:2] == b'\xff\xee' else "DIFOP" if d[:4] == b'\xa5\xff\x00\x5a' else "other"] += 1
print(c)
```

**Read the LiDAR's stored configuration** — capture one DIFOP frame and decode:
IP at bytes 10-13, destination IP 14-17, MAC 18-23, MSOP port `d[24]*256+d[25]`,
DIFOP port `d[26]*256+d[27]`, motor rpm `d[8]*256+d[9]`.
