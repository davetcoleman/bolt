cmake_minimum_required(VERSION 2.8.3)
project(curie_demos)

# C++ 11
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  moveit_boilerplate
  moveit_core
  moveit_visual_tools
  moveit_ompl
  rosparam_shortcuts
  ompl_visual_tools
  ompl_bolt
  tf_conversions
  descartes_core
  descartes_planner
  descartes_moveit
  ur5_demo_descartes
)

find_package(Boost REQUIRED)
find_package(Boost REQUIRED system serialization)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    moveit_boilerplate
    moveit_core
    moveit_visual_tools
    moveit_ompl
    rosparam_shortcuts
    ompl_visual_tools
    ompl_bolt
  DEPENDS
    Eigen
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_cart_path_planner
    ${PROJECT_NAME}_imarker_robot_state
    ${PROJECT_NAME}
)

###########
## Build ##
###########

include_directories(SYSTEM
  ${Boost_INCLUDE_DIR}
  ${EIGEN_INCLUDE_DIRS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Library
add_library(${PROJECT_NAME}
  src/curie_demos.cpp
  src/state_validity_checker.cpp
  src/moveit_base.cpp
  src/cart_path_planner.cpp
)
# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Main program
add_executable(${PROJECT_NAME}_curie_demos_main
  src/curie_demos_main.cpp
)
# Rename C++ executable without namespace
set_target_properties(${PROJECT_NAME}_curie_demos_main
  PROPERTIES OUTPUT_NAME curie_demos_main PREFIX "")
# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_curie_demos_main
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Demo for memory usage
# add_executable(${PROJECT_NAME}_memory_demo
#   src/tools/memory_demo.cpp
# )
# # Rename C++ executable without namespace
# set_target_properties(${PROJECT_NAME}_memory_demo
#   PROPERTIES OUTPUT_NAME memory_demo PREFIX "")
# # Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_memory_demo
#   #${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )

# Demo for distance function
add_executable(${PROJECT_NAME}_test_pose_distance
  src/tools/test_pose_distance.cpp
)
# Rename C++ executable without namespace
set_target_properties(${PROJECT_NAME}_test_pose_distance
  PROPERTIES OUTPUT_NAME test_pose_distance PREFIX "")
# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_test_pose_distance
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Testing ##
#############

## Test for correct C++ source code
roslint_cpp()

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_curie_demos_main
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

# Mark roslaunch files for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Mark config files for installation
# install(FILES config
#   FILES_MATCHING PATTERN "*.yaml"
#   PATTERN ".svn" EXCLUDE
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )