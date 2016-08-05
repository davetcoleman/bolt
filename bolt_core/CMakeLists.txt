cmake_minimum_required(VERSION 2.8.3)
project(ompl_bolt)

# C++ 11
add_compile_options(-std=c++11)

# Warnings
# add_definitions(-W -Wall -Wextra
#   -Wcast-qual -Wwrite-strings -Wunreachable-code -Wpointer-arith
#   -Winit-self -Wredundant-decls
#   -Wno-unused-parameter -Wno-unused-function)

find_package(catkin REQUIRED COMPONENTS
)

find_package(OMPL REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    ompl
  INCLUDE_DIRS
    src
  LIBRARIES
    ${PROJECT_NAME}
)

# User debug code only if not release
add_definitions(-DENABLE_DEBUG_MACRO)

###########
## Build ##
###########

include_directories(SYSTEM
  ${Boost_INCLUDE_DIR}
  ${OMPL_INCLUDE_DIRS}
)

include_directories(
  src
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

# Library
add_library(${PROJECT_NAME}
  src/ompl/tools/bolt/src/Bolt.cpp
  src/ompl/tools/bolt/src/BoltPlanner.cpp
  src/ompl/tools/bolt/src/SparseGraph.cpp
  src/ompl/tools/bolt/src/SparseGenerator.cpp
  src/ompl/tools/bolt/src/SparseCriteria.cpp
  src/ompl/tools/bolt/src/SparseStorage.cpp
  src/ompl/tools/bolt/src/TaskGraph.cpp
  src/ompl/tools/bolt/src/VertexDiscretizer.cpp
  src/ompl/tools/bolt/src/SamplingQueue.cpp
  src/ompl/tools/bolt/src/CandidateQueue.cpp
  #src/ompl/tools/bolt/src/PathSimplifier.cpp
)
# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${Boost_LIBRARIES}
)
