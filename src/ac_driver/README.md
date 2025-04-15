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

#### 2.1.4 Jetson Orin

For the Jetson Orin platform, install dependency library, for example:

```bash
sudo apt-get update
sudo apt-get install libavformat-dev libavdevice-dev libavcodec-dev
```

ensure that the CUDA environment is installed and the relevant dependencies are configured. Refer to the following link and commands for installing CUDA:
- [NVIDIA CUDA Toolkit](https://developer.nvidia.com/cuda-downloads)

After installation, ensure that the CUDA library paths are correctly configured, for example:
```bash
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
```

Then, follow the standard steps to build the project:
```bash
catkin_make
```

## 3. Usage

### 3.1 Prepare the ac_driver environment

Before starting the node, ensure that the `ROS_DOMAIN_ID` environment variable is set. Failing to set this may result in abnormal data delays or process issues. For example:
```bash
export ROS_DOMAIN_ID=67
```

For the compiled package, run the source command to set the node runtime environment. For the bash interpreter terminal and the zsh interpreter terminal, run the following commands respectively:
```bash
source devel/setup.bash 
```

```zsh
source devel/setup.zsh
```

### 3.2 Run the ac_driver Node

Run the following command to run the ac_driver node

```bash
roslaunch ac_driver start.launch 
#or
roscore 2>&1 >/dev/null &
rosrun ac_driver ms_node [_image_input_fps:=30 _imu_input_fps:=200 _enable_jpeg:=false]
```

#### Parameter Descriptions
- `_image_input_fps`: Sets the image frame rate. Supported values are 10Hz/15Hz/30Hz. The default value is 30Hz.
- `_imu_input_fps`: Sets the IMU frame rate. Supported values are 100Hz/200Hz. The default value is 200Hz.
- `_enable_jpeg`: Enables or disables JPEG encoding. `true` enables it, and `false` disables it. The default value is `false`.

For the Jetson Orin platform, it is recommended to adjust the parameters based on actual requirements to optimize performance.

The sensor supports setting the frame rates of images and Imu, where the frame rates supported by images include: 10Hz/15Hz/30Hz, The frame rates supported by Imu include: 100Hz/200Hz, default camera frame rate is 30Hz, The Imu frame rate is 200Hz. Depending on the method of starting the node, if the **rosrun** command is used to start as described in **Step2** above, parameters can be passed in through the command line (for example, "**_**image_input_fps:=30 **_**imu_input_fps:=200"); If using the **rosluanch** startup command, modify the startup parameter settings in the **start.luanch** file.

#### Format Description

- At present, image transcoding optimization has been done on rk3588 and jetson orin platforms, and the default driver is directly exported to nv12 and released into rgb24 format images through hardware;

- Other platforms use rgb24 by default and adopt cpu jepg compression. The compression function considers the impact of performance. The compression function is disabled by default and can be turned on by enable-jpeg switch.

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

