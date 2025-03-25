# ac_driver

[README](http://10.10.0.20/super_sensor_sdk/ros2_sdk/sdk_infra/-/blob/main/modules/ac_driver/README.md) | [中文文档](http://10.10.0.20/super_sensor_sdk/ros2_sdk/sdk_infra/-/blob/main/modules/ac_driver/README_CN.md)

## 1. Introduction

ac_driver is a AC sensor-driven ROS middleware node that receives sensor data, integrates and publishes it to other nodes for use. Sensor data includes cameras, liDAR and IMUs.

## 2. Installation

### 2.1 Build (Linux + ROS)

Ensure that the ROS noetic build environment is installed and that the entire project is developed and tested based on the ROS noetic version.

When the environment is ready, use the following command to download the code to the working directory.

```bash
# Using ssh
git clone git@github.com:RoboSense-Robotics/robosense_ac_ros_sdk_infra.git
# Using http
git clone https://github.com/RoboSense-Robotics/robosense_ac_ros_sdk_infra.git
```
Then, prepare system build environment base on Radxa and X86.

#### 2.1.1 Radxa Development Board

install ffmpeg-rockchip depend libs, refer to https://docs.radxa.com/rock5/rock5b/app-development/rtsp?target=ffmpeg:
```bash
sudo apt-get update
sudo apt-get install build-essential cmake git libdrm-dev librga-dev librockchip-mpp-dev libsdl2*-dev libx264-dev libx265-dev pkg-config
```
If you cannot access the apt source list to install the dependency libraries, please download the source code and install it. For reference, https://github.com/nyanmisaka/ffmpeg-rockchip/wiki/Compilation

install ffmpeg-rockchip:
```bash
git clone https://github.com/nyanmisaka/ffmpeg-rockchip
pushd ffmpeg-rockchip/
./configure --prefix=/usr --enable-gpl --enable-version3 --enable-libdrm --enable-rkmpp --enable-rkrga --enable-libx264 --enable-libx265 --enable-ffplay
make -j$(nproc)
sudo make install
popd
```

After the installation is complete, execute the following command to configure the system dependency library environment:
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
#### 2.1.2 X86 Board
install dependency library, for example:
```bash
sudo apt-get update
sudo apt-get install libavformat-dev libavdevice-dev libavcodec-dev
```

#### 2.1.3 Build
Then, enter the codepath directory, Run the following commands to compile:

```bash
catkin_make
```

## 3. Usage

### 3.1 Prepare the ac_driver environment
For the compiled package, run the source command to set the node runtime environment. For the bash interpreter terminal and the zsh interpreter terminal, run the following commands respectively
```bash
source devel/setup.bash 
```

### 3.2 Run the ac_driver Node
Run the following command to run the ac_driver node

```bash
roslaunch ac_driver start.launch
```

## 4. Features
### 4.1  Dependencies
The ac_driver node relies on several key libraries and packages to function properly. Here is a detailed list of the dependencies:

#### 4.1.2 robosense_msgs:
This custom ROS package defines the message formats for H.265 compressed images and other sensor data specific to the AC sensors. It is essential for the ac_driver node to interpret and publish the sensor data correctly.
### 4.2 Topic 
1. camera RGB image topic:/rs_camera/rgb
2. lidar topic:/rs_lidar/points
3. imu topic:/rs_imu
4. camera h265 video topic:/rs_camera/compressed

