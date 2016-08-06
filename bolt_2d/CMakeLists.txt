cmake_minimum_required(VERSION 2.8.3)
project(bolt_2d)

# C++ 11
add_compile_options(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  moveit_visual_tools
  roscpp
  rosparam_shortcuts
  bolt_core
  moveit_ompl
)

find_package(ompl REQUIRED)
find_package(Boost REQUIRED serialization)

include_directories(${Boost_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
    moveit_visual_tools
    roscpp
    rosparam_shortcuts
    bolt_core
    moveit_ompl
  INCLUDE_DIRS include
)

include_directories(
  include
  #/home/dave/ros/current/ws_moveit/src/ompl/src/  # TODO fix
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

## Build Library-----------------------

## Build Exexutables ------------------

# Lightning
# add_executable(lightning_demo
#   src/lightning_demo.cpp
# )
# target_link_libraries(lightning_demo
#   ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES}
# )

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

# # Spars
# add_executable(spars_demo
#   src/spars_demo.cpp
# )
# target_link_libraries(spars_demo
#   ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES}
# )

# # SPARS two
# add_executable(spars_two_demo
#   src/spars_two_demo.cpp
# )
# target_link_libraries(spars_two_demo
#   ${OMPL_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES}
# )
