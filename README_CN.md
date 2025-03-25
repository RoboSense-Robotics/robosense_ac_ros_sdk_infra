# ros_ac_sdk_infra  

[README](./README.md)|[中文文档](README_CN.md)

## 1. 简介

​	ros_ac_sdk_infra工程项目是基于ROS的包工程，基于该项目可以实现在ROS环境中，在接入RoboSense AC硬件即可可视化查看设备输出的点云、图像、IMU数据。


## 2. 前置依赖

- Ubuntu 20.04 (其他操作系统版本，需要适配)
- 操作系统匹配的ROS版本即可


## 3. 驱动安装和硬件连接

### 3.1 驱动安装(只需要安装一次) 

在当前工程目录下的scripts文件夹打开新的终端，安装如下图所示的命令执行脚本，执行安装驱动脚本，

```shell
# 进入源码下的脚本目录
cd /codepath/src/ac_driver/scripts
# 首次接入新设备执行（只需要一次）
sudo bash metaS_usb_permission.sh
```

等待安装，安装完成后，重启电脑即可。 

### 3.2 硬件连接

由于Robosense提供的设备数据传输量较大，所以要求必须使用USB3.0接入，连接后，可以使用usbview 工具查看进行进一步确认。



## 4. 编译ac_driver

- 步骤1: 打开新的终端，将终端路径切换到包含package.xml的src文件夹目录路径

- 步骤2:  执行如下编译命令

  ```shell
  # 进入源码根目录
  cd /codepath
  catkin_make
  ```

## 5. 运行ac_driver

- 步骤1: 对于编译好的包，执行source命令设置节点运行环境: 对于bash解释器终端和zsh解释器终端，分别运行如下对应命令

  - ```sh
    source devel/setup.bash 
    ```

  - ```shell
    source devel/setup.zsh 
    ```

- 步骤2: 启动节点: 

  ```sh
  roslaunch ac_driver start.launch 
  ```

  由于launch文件中配置了自动启动配置好的rviz工具，所以使用者无须单独启动rviz工具。


## 7. 话题名称及消息类型

- /rs_camera/rgb         ->  sensor_msgs::Image 
- /rs_lidar/points         -> sensor_msgs::PointCloud2, 其中点云的frame_id为"/rslidar"
- /rs_imu                      -> sensor_msgs::Imu  
