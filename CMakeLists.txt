cmake_minimum_required(VERSION 3.0.2)
project(lecture3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)
# catkin_python_setup()
# 需要加入 message_generation,必须有 std_msgs

## 配置 msg 源文件
add_message_files(
  FILES
  PosVel.msg
  PosVelArray.msg
)

# 生成消息时依赖于 std_msgs
generate_messages(
  DEPENDENCIES
  # std_msgs
)

#执行时依赖
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES demo02_talker_listener
  # CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
  # CATKIN_DEPENDS roscpp rospy message_runtime
   CATKIN_DEPENDS rospy message_runtime
#  DEPENDS system_lib
)

catkin_install_python(PROGRAMS 
scripts/environment.py
scripts/plan_env.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
