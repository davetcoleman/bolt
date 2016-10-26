cmake_minimum_required(VERSION 2.8.3)
project(bolt_moveit)

# C++ 11
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslint
  moveit_boilerplate
  moveit_dashboard
  moveit_core
  moveit_visual_tools
  rosparam_shortcuts
  bolt_core
  bolt_ros
  tf_conversions
  rostest
  roslint
)

find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system serialization)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    moveit_boilerplate
    moveit_core
    moveit_visual_tools
    rosparam_shortcuts
    bolt_core
    bolt_ros
  INCLUDE_DIRS
    ${EIGEN3_INCLUDE_DIR}
    include
  LIBRARIES
    ${PROJECT_NAME}
    ${PROJECT_NAME}_mbsp
)

# If enabled, show debug message output
add_definitions(-DENABLE_DEBUG_MACRO)

###########
## Build ##
###########

include_directories(SYSTEM
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

# link_directories(${OMPL_LIBRARY_DIRS})
# link_directories(${catkin_LIBRARY_DIRS})
# link_directories(${Boost_LIBRARY_DIRS})

# Model Based State Space
add_library(${PROJECT_NAME}_mbsp
  src/model_based_state_space.cpp
  src/threadsafe_state_storage.cpp
)
target_link_libraries(${PROJECT_NAME}_mbsp ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Library
add_library(${PROJECT_NAME}
  src/projection_viz_window.cpp
  src/moveit_viz_window.cpp
  src/state_validity_checker.cpp
  src/moveit_base.cpp
  src/cart_path_planner.cpp
)
# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_mbsp
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

if(CATKIN_ENABLE_TESTING)

  catkin_add_gtest(moveit_bolt_test test/moveit_bolt_test.cpp)
  target_link_libraries(moveit_bolt_test
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
  )
  #add_rostest(test/moveit_bolt_test.test)

endif()

## Test for correct C++ source code
roslint_cpp()

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_mbsp
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
