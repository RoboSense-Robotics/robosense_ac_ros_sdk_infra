cmake_minimum_required(VERSION 3.0.2)
project(robosense_msgs)

# 查找 catkin 和消息生成依赖
find_package(catkin REQUIRED COMPONENTS
  message_generation  # ROS 1 消息生成工具
  std_msgs
)

# 定义消息文件（假设消息文件在 msg/ 目录）
add_message_files(
  # DIRECTORY msg
  FILES
  RsCompressedImage.msg
  RslidarPacket.msg
  RslidarScans.msg
)

# 生成消息头文件
generate_messages(
  DEPENDENCIES
  std_msgs
)

# 声明 catkin 包
catkin_package(
  CATKIN_DEPENDS
  # message_runtime
  std_msgs
)

# 可选：安装消息文件
# install(DIRECTORY msg/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg
# )
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
