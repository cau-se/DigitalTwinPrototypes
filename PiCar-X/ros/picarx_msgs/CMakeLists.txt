cmake_minimum_required(VERSION 3.0.2)
project(picarx_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

catkin_python_setup()

add_message_files(
	DIRECTORY msg
    FILES
    DCMotor.msg
)


generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
