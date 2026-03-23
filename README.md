
## Setup 
- [WSL](https://docs.px4.io/main/en/dev_setup/dev_env_windows_wsl)
- [ROS](https://docs.px4.io/main/en/ros2/user_guide)
- [Gazebo](https://gazebosim.org/docs/latest/ros_installation/)
- Clone this repo into ros2 workspace `src` folder
- Copy contents of `models` folder into `PX4-Autopilot/Tools/simulation/gz/models`

### Our specific setup
- Ubuntu 22.04 LTS
- ROS2 Humble
- Gazebo Harmonic
- PX4 Autopilot 1.16
- px4_msgs 1.16

| QGC Parameters | Description |
|---|---|
| COM_RC_IN_MODE = 5 | RC > MAVL for manual control |
| COM_RC_OVERRIDE = 3 | Enable override during offboard mode |
| COM_OBL_RC_ACT = 2 | Return mode |

```bash
pkill -f gz; cd ~/drone_dev/ros2_ws && colcon build --packages-select drone_bringup drone_interface && source install/setup.bash && ros2 launch drone_bringup system.launch.py use_sim:=true
```

### TODO
- [x] Move everyting to separete dev folder and test
- [x] Add the custom drone model to repo
- [ ] Add custom world
- [ ] Add setup instructions to repo
- [x] Upload to github
- [ ] Improve simulation SLAM
- [ ] Add navigation
- [ ] Add dynamic map
- [ ] Make compatible with real drone
- [ ] Try HITL
- [ ] Setup raspberry PI as hotspot for direct SSH
- [ ] Connect to QGroundControl wirelessly
- [ ] adjust 
- [ ] Sensor integration