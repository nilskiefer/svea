# Development Guide

## Microcontroller development

This workflow lets you talk to a PX4 microcontroller from both QGroundControl
and ROS 2 on macOS by splitting one serial MAVLink stream into two UDP outputs.

### Prerequisites

- PX4 flight controller connected over USB serial.
- QGroundControl installed.
- SVEA container running with Docker Compose.

### Start the SVEA container

```bash
cd /Users/nils/Documents/GitHub/ITRL/svea
docker compose up -d
docker compose exec svea bash
```

By default, `docker compose up` now starts all runtime processes automatically in the container:

- `mavproxy.py` (serial -> UDP splitter)
- `mavros_node`
- `svea_core px4_uorb_tunnel`

If any one of these exits, the container exits.

Inside the container (when needed after source changes):

```bash
cd /svea_ws
colcon build --symlink-install
source /svea_ws/install/setup.bash
```

## Developing on macOS

### MAVProxy setup (venv)

From the repo root:

```bash
cd /Users/nils/Documents/GitHub/ITRL/svea
python3 -m venv .venv
source .venv/bin/activate
pip install future gnureadline MAVProxy
```

### Run MAVLink splitter

Use your working serial settings (`usbmodem01`, `57600`):

```bash
mavproxy.py \
  --master=/dev/tty.usbmodem01,57600 \
  --out=udp:127.0.0.1:14550 \
  --out=udp:127.0.0.1:14551
```

Linux/container copy-paste (auto-detect FCU device, hard-fail if missing):

```bash
DEV="$(ls -1 /dev/serial/by-id/* 2>/dev/null | head -n1)"; [ -n "$DEV" ] || DEV="$(ls -1 /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | head -n1)"; [ -n "$DEV" ] || { echo "No FCU serial device found"; exit 1; }; mavproxy.py --master="${DEV},57600" --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551
```

Notes:

- `14550` is typically consumed by QGroundControl.
- Use `14551` for your ROS 2 MAVLink bridge in Docker.
- From inside the container, connect to `host.docker.internal:14551`.

### Run MAVROS + PX4 uORB tunnel bridge manually (optional)

Inside the container, start MAVROS with SVEA params:

```bash
ros2 run mavros mavros_node --ros-args --params-file /svea_ws/src/svea_core/params/svea_mavros.yaml
```

In a second shell, start the PX4 uORB TUNNEL bridge:

```bash
ros2 run svea_core px4_uorb_tunnel
```

On PX4 NSH, ensure the PX4 uORB tunnel stream is enabled on the MAVLink instance used by
your USB link (usually instance 0):

```sh
mavlink stream -u 0 -s PX4_UORB_TUNNEL -r 20
```

Check output:

```bash
ros2 topic echo /px4/uorb_tunnel/frame
ros2 topic echo /px4/uorb/power_monitor
ros2 topic echo /px4/uorb/gpio_in
```

### If port 14550 is already in use

If you see `Address already in use`, quit MAVProxy and restart with a different
output port (for example, `14552`) for one of the endpoints.

## ROS manual control (steering, throttle, AUX servos)

This project uses MAVLink `MANUAL_CONTROL` via MAVROS topic:

- `/mavros/manual_control/send`

### Switch-gated behavior

Authority is selected by CH5 mode switch in PX4 firmware:

- `~1000` (low): accepts MAVLink manual control from ROS.
- `~1500` (mid): rejects MAVLink manual control (RC-only).
- `~2000` (high): kill.

### Field mapping and ranges

For `mavros_msgs/msg/ManualControl`:

- `y`: steering (`-1000..1000`)
- `z`: throttle (`0..1000`, where `500` is neutral)
- `aux1..aux6`: extra manual channels (`-1000..1000`, MAVLink v2 extensions)
- `enabled_extensions`: must enable aux fields (`252` enables aux1..aux6)

Important: publish continuously (10-20 Hz) and publish full state every message.
If a later message sets `aux1=0`, output moves to 0 (it does not latch previous 1000).

### Current output mapping (board defaults)

PCA9685:

- CH0: throttle (`Motor1`, function `101`)
- CH1: steering (`Servo1`, function `201`)
- CH2: `RC_AUX1` (function `407`) front diff
- CH3: `RC_AUX2` (function `408`) rear diff
- CH4: `RC_AUX3` (function `409`) gear
- CH5: `RC_AUX4` (function `410`) misc
- CH6: `RC_AUX5` (function `411`) misc

Binary endpoints configured for diff/gear:

- CH2/CH3/CH4 min/max = `1200/1800`

### Gear/diff polarity

`svea_lli_zephyr` used opposite front/rear differential pulses and an inverted
gear convention (`high_gear=true` => lower pulse). This setup keeps that semantic:

- Diff ON:
  - front diff (`aux1`) = `+1000` (high pulse on CH2)
  - rear diff (`aux2`) = `-1000` (low pulse on CH3)
- Diff OFF:
  - front diff (`aux1`) = `-1000`
  - rear diff (`aux2`) = `+1000`
- Gear HIGH (`high_gear=true`): `aux3 = -1000` (low pulse on CH4)
- Gear LOW (`high_gear=false`): `aux3 = +1000` (high pulse on CH4)

For any binary channel:

- `aux=-1000` -> channel min PWM
- `aux=+1000` -> channel max PWM

### Test commands

Steer right, small forward throttle, front diff ON, rear diff OFF, gear HIGH:

```bash
ros2 topic pub -r 20 /mavros/manual_control/send mavros_msgs/msg/ManualControl "{x: 0, y: 1000, z: 600, r: 0, buttons: 0, buttons2: 0, enabled_extensions: 252, s: 0, t: 0, aux1: 1000, aux2: -1000, aux3: -1000, aux4: 0, aux5: 0, aux6: 0}"
```

Neutral steering/throttle, all binary aux set to `-1000`:

```bash
ros2 topic pub -r 20 /mavros/manual_control/send mavros_msgs/msg/ManualControl "{x: 0, y: 0, z: 500, r: 0, buttons: 0, buttons2: 0, enabled_extensions: 252, s: 0, t: 0, aux1: -1000, aux2: -1000, aux3: -1000, aux4: 0, aux5: 0, aux6: 0}"
```

One-shot neutral:

```bash
ros2 topic pub -1 /mavros/manual_control/send mavros_msgs/msg/ManualControl "{x: 0, y: 0, z: 500, r: 0, buttons: 0, buttons2: 0, enabled_extensions: 252, s: 0, t: 0, aux1: -1000, aux2: -1000, aux3: -1000, aux4: 0, aux5: 0, aux6: 0}"
```

## WASD demo teleop (ROS -> MAVROS MANUAL_CONTROL)

A keyboard demo script is available at:

- `src/svea_examples/scripts/manual_control_wasd.py`

Behavior:

- publishes `mavros_msgs/msg/ManualControl` at 20 Hz
- full-state publish every cycle (steering, throttle, aux1..aux6)
- default `diff_on = true`
- diff/gear polarity follows the table above (old LLI-compatible)

Build once after adding the script:

```bash
cd /svea_ws
colcon build --symlink-install --packages-select svea_examples
source /svea_ws/install/setup.bash
```

Run:

```bash
ros2 run svea_examples manual_control_wasd.py
```

Keys:

- `w/s` (or arrow up/down): throttle up/down
- `a/d` (or arrow left/right): steering left/right
- `c`: center steering
- `v`: throttle neutral (`z=500`)
- `f`: toggle differential lock (front+rear with opposite pulse polarity)
- `g`: toggle high gear
- `1`/`2`: toggle aux4/aux5
- `r`: reset defaults
- `h`: print help
- `q`: quit
