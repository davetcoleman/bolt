cmake_minimum_required(VERSION 2.8.3)
project(bolt_ur5)

## Find catkin macros and libraries
find_package(catkin REQUIRED descartes_core descartes_moveit pluginlib moveit_core roscpp tf_conversions ur_kinematics)

find_package(rosconsole_bridge REQUIRED)
find_package(LAPACK REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    moveit_core
    descartes_moveit
    pluginlib
    DEPENDS
)

###########
## Build ##
###########


include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/ur5_robot_model.cpp
  src/ur_kin.cpp
  src/ur_moveit_plugin.cpp
  src/plugin_init.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${Eigen_LIBRARIES}  ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})

#############
## Install ##
#############


## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)