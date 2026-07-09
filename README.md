# livox_mvs_ros

***Simple and easy-to-use ROS drivers for Livox and Hikvision devices.***

**Supported Platforms**

- ROS-Noetic on Ubuntu20.04
- ROS-One on Ubuntu22.04


**SDK Packages**

- **livox_ros_driver**: for Livox AVIA.
- **livox_ros_driver2**: for Livox MID360.
- **mvs_ros_driver**: for Hikvision MV-CU013-A0UC.
- **gnss_ros_driver**: for GNSS/RTK UM982.

## Third-party

- [Livox-SDK](https://github.com/Livox-SDK/Livox-SDK): for Livox AVIA.
- [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2): for Livox MID360.


## Run

```bash
mkdir -p ws_sensor/src
cd ws_sensor/src
git clone git@github.com:zhan994/livox_mvs_ros.git
cd livox_mvs_ros/livox_ros_driver2
./build.sh ROS1

cd ../../..
source devel/setup.bash
roslaunch start_all start_mvs_mid360.launch

# for gnss_ros_driver
# sudo chmod 777 <serial_port>
roslaunch gnss_ros_driver gnss_ros_driver.launch
```

## Related Work

- [LIV_handhold: LiDAR_Inertial_Visual_Handhold](https://github.com/xuankuzcr/LIV_handhold)
- [FAST-Calib: LiDAR-Camera Extrinsic Calibration in One Second](https://github.com/hku-mars/FAST-Calib)
- [FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry](https://github.com/hku-mars/FAST-LIVO2)