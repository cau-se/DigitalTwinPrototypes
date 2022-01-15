cmake_minimum_required(VERSION 3.0.2)
project(picarx_ackermann_drive)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  picarx_msgs
)


#catkin_python_setup()
catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS
  nodes/ackermann_drive_skill.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
