# pc_trunk_ws — PC workspace

**Platform:** Ubuntu 22.04, ROS 2 Humble, CycloneDDS (`rmw_cyclonedds_cpp`)

## Packages

| Package | Description |
|---|---|
| `trunk_description` | URDF/xacro model of the trunk |
| `trunk_bringup` | Launch files that start the system and RViz2 |
| `trunk_control` | Controllers for the three tendons/steppers |
| `trunk_sim` | MuJoCo pseudo-rigid-body simulation |

## Prerequisites

**OS:** Ubuntu 22.04 (ROS 2 Humble does not install from apt on 24.04)

1. **ROS 2 Humble** — follow docs.ros.org → Humble → Installation → Ubuntu (deb packages), then:
   ```bash
   sudo apt install ros-humble-desktop ros-dev-tools
   ```
2. **ROS packages used by this workspace:**
   ```bash
   sudo apt install ros-humble-xacro ros-humble-robot-state-publisher \
     ros-humble-joint-state-publisher-gui ros-humble-rmw-cyclonedds-cpp
   ```
3. **Python libraries:**
   ```bash
   sudo apt install python3-serial python3-pip   # serial link to the Arduino (hardware only)
   pip install mujoco                            # trunk_sim physics engine
   ```
4. **rosdep** (first time on a computer only):
   ```bash
   sudo rosdep init
   rosdep update
   ```

## Get the code

Clone into your home folder:
```bash
cd ~
git clone git@github.com:aamhegazy/xcr_continuum_robot.git
```
Only `pc_trunk_ws` needs building on a PC; `rpi_trunk_ws` is for the Raspberry Pi.

## Build

```bash
cd ~/xcr_continuum_robot/pc_trunk_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y   # installs dependencies (needs internet)
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
# robot model only in RViz2 — use this first to check the install
ros2 launch trunk_description display.launch.py

# full system
ros2 launch trunk_bringup trunk.launch.py

# system with RViz2
ros2 launch trunk_bringup trunk_rviz.launch.py
```

## Talking to the Pi

Both machines must use the same middleware and domain ID:
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=<same number on PC and Pi>
```
