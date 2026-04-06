
## Setup 
- [WSL](https://docs.px4.io/main/en/dev_setup/dev_env_windows_wsl)
- [ROS](https://docs.px4.io/main/en/ros2/user_guide)
- [Gazebo](https://gazebosim.org/docs/latest/ros_installation/)
- Clone this repo into ros2 workspace `src` folder
- https://github.com/ros/sdformat_urdf

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
EKF2_GPS_CTRL: 1 # enable GPS fusion
EKF2_HGT_REF: 0 # use baro as height reference initially
EKF2_BARO_CTRL: 1     # always want baro
EKF2_MAG_TYPE: 1      # always want mag fusion
EKF2_MULTI_IMU: 1     # dual IMU on real FC
param set SENS_IMU_AUTOCAL 0    # disable auto-calibration checks
param set COM_ARM_IMU_ACC 1.0   # relax accel check threshold
Tune EKF2 to expect higher noise (keeps realistic noise)
bashparam set EKF2_ACC_NOISE  0.5   # default 0.35
param set EKF2_ABL_NOISE  0.02  # default 0.01
param set EKF2_ABL_LIM    0.8   # default 0.4
param set EKF2_ABL_TAU    0.5   # default 0.4, slower bias learning

Parameters for simulation
UXRCE_DDS_SYNCT 0

env 
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0

### Build and run SITL simulation
- Run QGroundControl
- Run riv2
- Goto ros2 workspace and run:
```bash
colcon build --packages-select drone_bringup drone_interface && source install/setup.bash && ros2 launch drone_bringup sim.launch.py
```
or
```bash
pkill -9 -f gz; colcon build --packages-select drone_bringup drone_interface && source install/setup.bash && ros2 launch drone_bringup sim.launch.py
```
### Run on real hardware
- Run QGroundControl
- Run riv2
- Goto ros2 workspace and run:
```bash
ros2 launch drone_bringup real.launch.py
```

### TODO
- [x] Move everyting to separete dev folder and test
- [x] Add the custom drone model to repo
- [ ] Add custom world
- [ ] Get turtlebot3_house working
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
- [ ] path env variable for px4 and qgc
- [ ] make urdf for robot state pub to replae static tf