cmake_minimum_required(VERSION 3.0.2)
project(alexa_ros_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()


catkin_package()

catkin_install_python(PROGRAMS
  scripts/alexa_interface.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

