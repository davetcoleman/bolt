cmake_minimum_required(VERSION 2.8.3)
project(moveit_ompl)

# C++ 11
add_compile_options(-std=c++11) 

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED system filesystem date_time thread serialization)
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  moveit_visual_tools
  rosparam_shortcuts
  roscpp
  rosconsole
  pluginlib
  tf
  eigen_conversions
  sensor_msgs
)
find_package(OMPL REQUIRED)

catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
    ${OMPL_LIBRARIES}
  INCLUDE_DIRS
    include
    ${OMPL_INCLUDE_DIRS}
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
)

include_directories(SYSTEM
                    ${Boost_INCLUDE_DIRS})

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${OMPL_INCLUDE_DIRS})

link_directories(${OMPL_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

add_library(${PROJECT_NAME}
  # src/planning_context_manager.cpp
  # src/model_based_planning_context.cpp
  src/model_based_state_space.cpp
  src/detail/threadsafe_state_storage.cpp
  src/remote_control.cpp
  # src/detail/state_validity_checker.cpp
  # src/detail/projection_evaluators.cpp
  # src/detail/goal_union.cpp
  # src/detail/constrained_sampler.cpp
  # src/detail/constrained_valid_state_sampler.cpp
  # src/detail/constrained_goal_sampler.cpp
  # src/detail/ompl_console.cpp
)
target_link_libraries(${PROJECT_NAME} ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# add_library(ompl_planner_manager src/ompl_planner_manager.cpp)
# target_link_libraries(ompl_planner_manager ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# install(TARGETS ${PROJECT_NAME} ompl_planner_manager
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
# install(DIRECTORY include/ DESTINATION include)


# install(FILES moveit_ompl_plugin_description.xml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
