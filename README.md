## Instructions  
- [WSL](https://docs.px4.io/main/en/dev_setup/dev_env_windows_wsl)
- [ROS](https://docs.px4.io/main/en/ros2/user_guide)
- [Gazebo](https://gazebosim.org/docs/latest/ros_installation/)
- Clone this repo into ros2 workspace `src` folder
- Set QGroundControl parameters

## Resources
- [Using PX4's Navigation Filter (EKF2) ](https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf#using-px4-s-navigation-filter-ekf2)
- [Indoor Configuration](https://docs.px4.io/main/en/advanced_config/tuning_the_ecl_ekf#typical-configurations)

## Setup
- Ubuntu 22.04 LTS
- ROS2 Humble
- Gazebo Harmonic
- PX4 Autopilot 1.16
- px4_msgs 1.16 

## FC Parameters
| QGC Parameters        | Description                           |
| ---                   | ---                                   |
| COM_RC_IN_MODE = 5    | RC > MAVL for manual control          |
| COM_RC_OVERRIDE = 3   | Enable override during offboard mode  |
| COM_OBL_RC_ACT = 3    | Return mode on RC signal issue        |
| NAV_DLL_ACT = 2       | Return mode on ground station link loss|
| RTL_RETURN_ALT = 2.0  | Safe clearance height (e.g., 2.5 meters) that clears all local indoor obstacles |
| EKF2_GPS_CTRL = 0     | Disable GPS                           |
| EKF2_BARO_CTRL = 1    | Enable Barometer fusion               |
| EKF2_RNG_CTRL = 1     | Enable conditional range finder fusion during low speed/low altitude |
| EKF2_HGT_REF = 2      | Use range finder as height referance  |
| EKF2_EV_CTRL = 9      | Fuses HPos, Yaw from LIDAR SLAM       |
| EKF2_OF_CTRL = 1      | Fuses Optical Flow data               |   
| EKF2_MAG_TYPE = 5     | Completely ignore magnetometer        |
| SENS_IMU_MODE = 0     | allows both raw IMU streams to pass   |
| EKF2_MULTI_IMU = 2 (1 for sim) | Enable dual-IMU multi-instance EKF redundancy |
| EKF2_EV_DELAY = 50 (0 for sim) | processing delay of SLAM (**Adjust**. Start high (100), tune down later) |
| EKF2_EV_POS_X,Y,Z     | LIDAR distance from physical center of the FC |
| EKF2_OF_POS_X,Y,Z 0.2,0.0,0.2 | Optical flow sensor positoin  |
| UXRCE_DDS_SYNCT = 1 (0 for sim) | Micro-XRCE-DDS Time Synchronization |
| COM_ARM_WO_GPS = 2    | Allows arming without a GPS home lock |
| MPC_ALT_MODE = 0      | Fixed Altitude (not Terrain following)|
| SYS_HAS_MAG = 0       |                                       |

### Need to test
| QGC Parameters        | Description                           |
| ---                   | ---                                   |
| SENS_EN_PMW3901 = 1   | Enable PMW3901 Optical Flow           |
| SENS_FLOW_ROT = 0     | Optical flow rotation (adjust)        |
| SENS_IMU_AUTOCAL = 1  | IMU calibration check (better to set 0 and manually perform a high-quality thermal and sensor calibration via QGC in a stable environment once, save it to avoid pre-flight error) |
| COM_ARM_IMU_ACC = 1   | Relax accel check threshold to allow arming on imperfect/vibrating hardware |
| COM_ARM_IMU_GYR = 0.25| Prevents gyro drift from blocking your flight |
| EKF2_ACC_NOISE = 0.5  | Tune EKF2 to expect higher noise (keeps realistic noise) |
| EKF2_ABL_NOISE = 0.02 | "                                     |
| EKF2_ABL_LIM = 0.8    | "                                     | 
| EKF2_ABL_TAU = 0.5    | "                                     |
| MAV_1_CONFIG =        | Enables the TELEM port on RPi         |
|                       | Indoor flight requires snappier, safer control limits ↓ |
| MPC_XY_VEL_MAX = 1.5 to 2.0 | Caps maximum horizontal speed for safe indoor tracking |
| MPC_Z_VEL_MAX_DN = 1.0| Caps descent speed to prevent ground-effect crashes | 
| MPC_XY_CRUISE = 1.0   | Sets the default autonomous cruise speed to a manageable speed |
| EKF2_RNG_A_HMAX = 3.5 | If rangefinder can read up to 4 meters, set it to 3.5 |
| EKF2_RNG_A_HMAX = 1.5 or 2.0 | Raise velocity limit to prevent accidental cutoffs during fast moves |

### Environment setup
```
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
```

### Build and run SITL simulation
- Run QGroundControl
- Run riv2
- Goto ros2 workspace and run:
```bash
colcon build --packages-select drone_bringup drone_interface drone_sim && source install/setup.bash && ros2 launch drone_sim sim.launch.py
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

## Preflight check
- Ensure your Optical Flow data is feeding separately into the flight controller via its native driver (I2C/UART/CAN) so it populates the standard OPTICAL_FLOW_RAD message natively
- Validate Before You Fly. Before testing a Return Mode command, place the drone on the floor and open QGroundControl: Switch to the Mavlink Inspector window. Verify that the LOCAL_POSITION_NED values match reality.

### TODO
#### Must
- [ ] Organise fc parameters 
- [ ] programitcally change fc parameters
- [ ] turn off gps
- [ ] Add navigation
- [ ] Add setup instructions to repo
- [ ] Sensor integration
- [ ] Make compatible with real drone
#### Optional
- [ ] modify Gazebo model file to include a second distinct IMU sensor plugin with its own ROS topic for EKF2_MULTI_IMU = 2
- [ ] add sensors in simulation
- [ ] Setup raspberry PI as hotspot for direct SSH
- [ ] Get turtlebot3_house working
- [ ] Improve simulation SLAM
- [ ] Add dynamic map
- [ ] Try HITL
- [ ] Connect to QGroundControl wirelessly
- [ ] make urdf for robot state pub to replae static tf