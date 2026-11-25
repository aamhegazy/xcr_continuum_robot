# pc_trunk_ws — PC workspace

**Platform:** Ubuntu 22.04, ROS 2 Humble, CycloneDDS (`rmw_cyclonedds_cpp`)

## Packages

| Package | Description |
|---|---|
| `trunk_description` | URDF/xacro model of the trunk |
| `trunk_bringup` | Launch files that start the system and RViz2 |
| `trunk_control` | Controllers for the three tendons/steppers |
| `trunk_sim` | MuJoCo pseudo-rigid-body simulation |

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
# full system
ros2 launch trunk_bringup trunk.launch.py

# system with RViz2
ros2 launch trunk_bringup trunk_rviz.launch.py

# robot model only in RViz2 (check the URDF)
ros2 launch trunk_description display.launch.py
```

## Talking to the Pi

Both machines must use the same middleware and domain ID:
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=<same number on PC and Pi>
```
