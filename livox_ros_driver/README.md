# Avia Sync

Avia 的 PPS + 串口 GPRMC 同步：

- 配置启用及串口参数：config/livox_lidar_config.json:22
- 初始化串口时间同步、注册回调：livox_ros_driver/lds_lidar.cpp:105
- 串口读取并筛选 $GPRMC / $GNRMC：timesync/timesync.cpp:125
- 将收到的 RMC 交给 Livox SDK 的 LidarSetRmcSyncTime()：livox_ros_driver/lds_lidar.cpp:538
- Avia 以 PPS-GPS 时间戳生成 Unix ns 时间：livox_ros_driver/lds.cpp:127
  

Avia 向相机传时间戳：
      
- 驱动启动时创建并映射共享文件：livox_ros_driver/livox_ros_driver.cpp:49、livox_ros_driver/livox_ros_driver.cpp:97
- 仅在当前帧 LivoxCustomMsg 的首个包时，写入 pointt->low = timestamp：livox_ros_driver/lddc.cpp:435