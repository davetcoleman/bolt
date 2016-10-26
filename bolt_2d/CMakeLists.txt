cmake_minimum_required(VERSION 2.8.3)
project(bolt_2d)

# C++ 11
add_compile_options(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  rviz_visual_tools
  moveit_dashboard
  roscpp
  rosparam_shortcuts
  bolt_core
  bolt_ros
  rostest
  roslint
)

find_package(ompl REQUIRED)
find_package(Boost REQUIRED serialization)

catkin_package(
  CATKIN_DEPENDS
    rviz_visual_tools
    moveit_dashboard
    roscpp
    rosparam_shortcuts
    bolt_core
    bolt_ros
  INCLUDE_DIRS include
)

# User debug code only if not release
add_definitions(-DENABLE_DEBUG_MACRO)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# Bolt Demo
add_executable(${PROJECT_NAME}
  src/bolt_2d.cpp
  src/two_dim_viz_window.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${Boost_LIBRARIES}
)

##########
## TEST ##
##########

if(CATKIN_ENABLE_TESTING)

  catkin_add_gtest(2d_bolt_test test/2d_bolt_test.cpp)
  target_link_libraries(2d_bolt_test
    #${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
  )

endif()

## Test for correct C++ source code
roslint_cpp()