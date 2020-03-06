cmake_minimum_required(VERSION 2.8.3)
project(dsr_robot_master)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  dsr_msgs  
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(PROGRAMS robot_master.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS robot/robot_a0509_master.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS robot/robot_a0509_slave.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS robot/robot_m1013_master.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS robot/robot_m1013_slave.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

