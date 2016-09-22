cmake_minimum_required(VERSION 2.8.3)
project(bolt_core)

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
  src/bolt_core/src/Bolt.cpp
  src/bolt_core/src/BoltPlanner.cpp
  src/bolt_core/src/SparseGraph.cpp
  src/bolt_core/src/SparseGenerator.cpp
  src/bolt_core/src/SparseCriteria.cpp
  src/bolt_core/src/SparseStorage.cpp
  src/bolt_core/src/TaskGraph.cpp
  src/bolt_core/src/VertexDiscretizer.cpp
  #src/bolt_core/src/SamplingQueue.cpp
  src/bolt_core/src/CandidateQueue.cpp
  src/bolt_core/src/SparseSmoother.cpp
  src/bolt_core/src/SPARS2.cpp
)

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${Boost_LIBRARIES}
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

# Mark cpp header files for installation
install(DIRECTORY src/bolt_core/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
