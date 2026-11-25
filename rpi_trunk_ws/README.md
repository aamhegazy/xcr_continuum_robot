# rpi_trunk_ws — Raspberry Pi workspace

**Platform:** Raspberry Pi 4 (`hegzo`, 10.42.0.239), Ubuntu 22.04, ROS 2 Humble, CycloneDDS

The Pi connects to the Arduino Due's **Native USB** port (`SerialUSB`,
usually `/dev/ttyACM0`) and forwards motor commands from ROS 2.
Your user must be in the `dialout` group: `sudo usermod -aG dialout $USER`, then log out and back in.

## Packages

| Package | Description |
|---|---|
| `trunk_control` | Node that talks to the Arduino Due and drives the three steppers |

## Files

- `cyclonedds.xml` — CycloneDDS settings for talking to the PC.
  On the Pi it lives at `~/cyclonedds.xml` and is used with
  `export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml`

## Motor wiring (Arduino Due)

| Motor | Due pins (IN1–IN4) | Motor type |
|---|---|---|
| 1 | 2, 3, 4, 5 | Moons' 5.8 Ω |
| 2 | 6, 7, 8, 9 | Moons' 5.8 Ω |
| 3 | 22, 24, 26, 28 | XT6187 27.4 Ω |

Step order: IN1, IN3, IN2, IN4 · 200 steps/rev

## Build

```bash
cd ~/xcr_continuum_robot/rpi_trunk_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 run trunk_control motor_bridge      # serial link to the Arduino Due
```

Other nodes in this package:

| Node | Purpose |
|---|---|
| `motor_bridge` | Sends step commands to the Arduino Due over SerialUSB |
| `kinematics_node` | Converts a target bend into tendon/motor positions |
| `motor_sliders` | GUI sliders to move each motor by hand |
| `trunk_shape` | Computes/publishes the trunk's shape |
