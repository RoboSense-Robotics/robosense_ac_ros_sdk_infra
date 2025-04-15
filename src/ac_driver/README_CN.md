# ac_driver

[README](http://10.10.0.20/super_sensor_sdk/ros2_sdk/sdk_infra/-/blob/main/modules/ac_driver/README.md) | [中文文档](http://10.10.0.20/super_sensor_sdk/ros2_sdk/sdk_infra/-/blob/main/modules/ac_driver/README_CN.md)

## 1. 简介

ac_driver是MetaS传感器驱动的ROS中间件节点,用于接收传感器数据，整合和发布给其它节点使用。传感器数据包括摄像头，激光雷达和IMU。

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

#### 2.1.4 Jetson Orin

对于Jetson Orin平台，安装依赖库,如下示例:

```shell
sudo apt-get update
sudo apt-get install libavformat-dev libavdevice-dev libavcodec-dev
```

对于Jetson Orin平台，确保已安装CUDA环境，并配置相关依赖库。以下是安装CUDA的参考链接和命令：
- [NVIDIA CUDA Toolkit](https://developer.nvidia.com/cuda-downloads)

安装完成后，确保CUDA库路径已正确配置，例如：
```bash
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
```

然后按照常规步骤进行编译：
```bash
catkin_make
```

## 3. 运行

### 3.1 准备环境

在启动节点之前，请确保已设置环境变量`ROS_DOMAIN_ID`，否则可能导致数据延迟异常或进程异常。例如：
```bash
export ROS_DOMAIN_ID=67
```

对于编译好的包，执行source命令设置节点运行环境: 对于bash解释器终端和zsh解释器终端，分别运行如下对应命令：
```bash
source devel/setup.bash 
```

```bash
source devel/setup.zsh 
```

### 3.2 运行ac_driver节点

使用以下命令运行ac_driver节点

```bash
roslaunch ac_driver start.launch 
#或者 
roscore 2>&1 >/dev/null &
rosrun ac_driver ms_node [_image_input_fps:=30 _imu_input_fps:=200 _enable_jpeg:=false]
```

#### 参数说明
- `_image_input_fps`: 设置图像帧率，支持的值为10Hz/15Hz/30Hz，默认值为30Hz。
- `_imu_input_fps`: 设置IMU帧率，支持的值为100Hz/200Hz，默认值为200Hz。
- `_enable_jpeg`: 是否启用JPEG编码，`true`表示启用，`false`表示禁用，默认值为`false`。

对于Jetson Orin平台，建议根据实际需求调整参数以优化性能。

传感器支持设置图像和Imu的帧率，其中图像支持的帧率包含: 10Hz/15Hz/30Hz, Imu支持的帧率包含: 100Hz/200Hz，默认相机帧率为30Hz, Imu帧率为200Hz, 根据启动节点的方法不同，如果前述1/2所述，如果通过rosrun命令启动，则可以通过命令行形式传入参数(例如"_image_input_fps:=30 _imu_input_fps:=200")； 如果通过rosluanch 启动命令，则修改start.luanch文件中的启动参数设置。

#### 格式说明

- 当前已对rk3588、jetson orin平台对图像做了转码优化，默认驱动直接出nv12，通过硬件转成rgb24格式图像发布；

- 其他平台默认直出rgb24，并采用cpu jepg压缩，压缩功能考虑到性能的影响，默认关闭压缩功能，可以通过enable-jpeg开关打开。

## 4. 特性
### 4.1  依赖
ac_driver节点依赖以下关键的库和软件包:

#### 4.1.2 robosense_msgs:
为H.265定制的ROS消息，用于传输metaS传感器的图像数据。

### 4.2 Topic 
1. camera RGB image topic:/rs_camera/rgb
2. lidar topic:/rs_lidar/points
3. imu topic:/rs_imu
4. camera h265 video topic:/rs_camera/compressed

