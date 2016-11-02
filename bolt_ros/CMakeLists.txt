cmake_minimum_required(VERSION 2.8.3)
project(bolt_ros)

# C++ 11
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosparam_shortcuts
  roslib
  bolt_core
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    roslib
    bolt_core
)

###########
## Build ##
###########

include_directories(SYSTEM
  ${OMPL_INCLUDE_DIRS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Library
add_library(${PROJECT_NAME}
  src/ompl_rosparam.cpp
)
# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)