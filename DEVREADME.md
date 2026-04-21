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

### Run MAVROS + PX4 uORB tunnel bridge in container

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
