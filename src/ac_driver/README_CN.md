# ac_driver

[README](http://10.10.0.20/super_sensor_sdk/ros2_sdk/sdk_infra/-/blob/main/modules/ac_driver/README.md) | [中文文档](http://10.10.0.20/super_sensor_sdk/ros2_sdk/sdk_infra/-/blob/main/modules/ac_driver/README_CN.md)

## 1. 简介

ros_metas是MetaS传感器驱动的ROS中间件节点,用于接收传感器数据，整合和发布给其它节点使用。传感器数据包括摄像头，激光雷达和IMU。

## 2. 构建

### 2.1 编译 (Linux + ROS)

确保ROS noetic的编译环境已安装，整个工程的开发和测试基于的是ROS noetic版本。

环境准备好后，使用以下命令将代码下载到工作目录下。

```bash
# Using ssh
git clone git@github.com:RoboSense-Robotics/robosense_ac_ros_sdk_infra.git
# Using http
git clone https://github.com/RoboSense-Robotics/robosense_ac_ros_sdk_infra.git
```
Then, prepare system build environment base on Radxa and X86.
然后进行系统编译依赖的准备操作，如下基于Radxa开发板及X86环境.

#### 2.1.1 Radxa开发板
参考:https://docs.radxa.com/rock5/rock5b/app-development/rtsp?target=ffmpeg
安装ffmpeg-rockchip依赖库,执行以下命令:

```bash
sudo apt-get update
sudo apt-get install build-essential cmake git libdrm-dev librga-dev librockchip-mpp-dev libsdl2*-dev libx264-dev libx265-dev pkg-config
```
注意如果无法访问源安装依赖库，请下载源码安装，参考:https://github.com/nyanmisaka/ffmpeg-rockchip/wiki/Compilation

安装ffmpeg-rockchip:

```bash
git clone https://github.com/nyanmisaka/ffmpeg-rockchip
pushd ffmpeg-rockchip/
./configure --prefix=/usr --enable-gpl --enable-version3 --enable-libdrm --enable-rkmpp --enable-rkrga --enable-libx264 --enable-libx265 --enable-ffplay
make -j$(nproc)
sudo make install
popd
```
在安装完成后，执行以下命令配置系统依赖库环境:

```bash
sudo ln -s /usr/lib/aarch64-linux-gnu/librga.so.2.1.0 /usr/lib/aarch64-linux-gnu/librga.so
sudo ln -s /usr/lib/aarch64-linux-gnu/libdrm.so.2.123.0 /usr/lib/aarch64-linux-gnu/libdrm.so
sudo rm /usr/lib/aarch64-linux-gnu/libavformat.*
sudo rm /usr/lib/aarch64-linux-gnu/libavutil.*
sudo rm /usr/lib/aarch64-linux-gnu/libswscale.*
sudo rm /usr/lib/aarch64-linux-gnu/libpostproc.*
sudo rm /usr/lib/aarch64-linux-gnu/libavdevice.*
sudo rm /usr/lib/aarch64-linux-gnu/libswresample.*
sudo rm /usr/lib/aarch64-linux-gnu/libavfilter.*
sudo rm /usr/lib/aarch64-linux-gnu/libavcodec*
```
#### 2.1.2 X86
安装依赖库,如下示例:
```bash
sudo apt-get update
sudo apt-get install libavformat-dev libavdevice-dev libavcodec-dev
```

#### 2.1.3 编译
然后进入codepath目录, 使用以下命令进行编译:

```bash
catkin_make
```
## 3. 运行

### 3.1 准备环境

对于编译好的包，执行source命令设置节点运行环境: 对于bash解释器终端和zsh解释器终端，分别运行如下对应命令

```bash
source devel/setup.bash 
```

### 3.2 运行ros_metas节点
使用以下命令运行ros_metas节点

```bash
roslaunch ac_driver start.launch
```



## 4. 特性
### 4.1  依赖
ros_metas节点依赖以下关键的库和软件包:

#### 4.1.2 robosense_msgs:
为H.265定制的ROS消息，用于传输metaS传感器的图像数据。

### 4.2 Topic 
1. camera RGB image topic:/rs_camera/rgb
2. lidar topic:/rs_lidar/points
3. imu topic:/rs_imu
4. camera h265 video topic:/rs_camera/compressed

