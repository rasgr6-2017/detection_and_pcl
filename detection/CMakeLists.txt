cmake_minimum_required(VERSION 2.8.3)
project(simple_detection)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
)

catkin_install_python(PROGRAMS
   src/image_converter.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

