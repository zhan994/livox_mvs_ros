# livox_mvs_ros

***Simple and easy-to-use ROS drivers for Livox and Hikvision devices.***

***This branch is used for ROS-One on Ubuntu22.04.***

- **livox_ros_driver**: for Livox AVIA.
- **livox_ros_driver2**: for Livox MID360.
- **mvs_ros_driver**: for Hikvision MV-CU013-A0UC.
- **gnss_ros_driver**: for GNSS/RTK UM982.

## Run

```bash
mkdir -p ws_sensor/src
cd ws_sensor/src
git clone git@github.com:zhan994/livox_mvs_ros.git
cd livox_mvs_ros/livox_ros_driver2
./build.sh ROS1
```
