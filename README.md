# ros_ac_sdk_infra

[README](./README.md)|[中文文档](README_CN.md)

## 1. Overview

​	The ros_ac_sdk_infra project is a ROS-based package project. Based on this project, the point cloud, image and IMU data output of the device can be visually viewed by accessing RoboSense AC hardware in ROS environment.

## 2. Prerequisites 

- Ubuntu20.04 

- ROS which is matched the system version 


## 3. Drive Install And Hardware Connect 

### 3.1 Drive Install(Only Need Once)  

Open a new terminal in the scripts folder in the current project directory, install the command execution script as shown in the following figure, and run the installation driver script.

```shell
# Go to the script directory under source code
cd /codepath/src/ros_metas/scripts
# First access to a new device execution (only once required)
sudo bash metaS_usb_permission.sh
```

Wait for the installation, and restart the computer.

### 3.2 Hardware Connect 

Due to the large amount of data transmission of the device provided by Robosense, it is required to use USB3.0 access. 

## 4. Build 

- Step1: Open a teminal And change the work path to the "src" directory of this ROS package 

- Step2: Run ros build command 

  ```shell
  # Go to the source code root directory
  cd /codepath
  catkin_make
  ```

## 5. Run 

- Step1: Setup the node's environment by source command: 

  - `source devel/setup.bash`(For Bash Terminal) 
  - `source devel/setup.zsh`(For Zsh Termial) 

- Step2: Run node: 

  ```shell
  roslaunch metas_ros start.launch 
  ```

  **Notice:  As the "start.launch" setting the rviz node to run, so the user not need to start rviz by youself**. 
  
  

## 7. Topic Name And Data Type 

- /rs_camera/rgb     ->  sensor_msgs::Image 
- /rs_lidar/points     -> sensor_msgs::PointCloud2, the pointcloud frame_id is **"/rslidar"**
- /rs_imu                  -> sensor_msgs::Imu  
