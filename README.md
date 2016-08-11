# BOLT

A new planner based on SPARS2.

Developed by [Dave Coleman](http://dav.ee/) at the University of Colorado Boulder in collaboration with ROS Industrial, Southwest Research Institute, and the National Institute of Standards and Technology.

Status:

 * [![Travis Build Status](https://travis-ci.org/davetcoleman/bolt.svg)](https://travis-ci.org/davetcoleman/bolt) Travis - Continuous Integration

![](bolt_hilgendorf/resources/screenshot.png)

### Build from Source

Currently tested for ROS Indigo. To build this package, create a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

    wstool init .
    wstool merge https://raw.githubusercontent.com/davetcoleman/bolt/jade-devel/bolt.jade.rosinstall
    wstool update
    wget https://raw.githubusercontent.com/ros-gbp/ompl-release/debian/kinetic/xenial/ompl/package.xml ompl/
    sudo apt-get remove ros-jade-ompl
    touch robotiq/robotiq_action_server/CATKIN_IGNORE
    touch robotiq/robotiq_c_model_control/CATKIN_IGNORE
    touch robotiq/robotiq_ethercat/CATKIN_IGNORE
    touch robotiq/robotiq_s_model_control/CATKIN_IGNORE
    touch robotiq/robotiq_joint_state_publisher/CATKIN_IGNORE
    touch universal_robot/ur_gazebo/CATKIN_IGNORE
    rosdep install -y --from-paths . --ignore-src --rosdistro jade
    cd ..
    catkin config --extend /opt/ros/jade --cmake-args -DCMAKE_BUILD_TYPE=Release
    catkin build

## Run in Docker

Run with just terminal:

    docker run -it davetcoleman/bolt:bolt-jade-build

Run with GUI:

    # This is not the safest way however, as you then compromise the access control to X server on your host
    xhost +local:root # for the lazy and reckless
    docker run -it --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" davetcoleman/bolt:bolt-jade-build
    export containerId=$(docker ps -l -q)
    # Close security hole:
    xhost -local:root

## Layout

- bolt_core: The core algorithm for sparse roadmap creation and multi-model task planning is locate
- bolt_2d: A simple 2d/3d toy problem for testing bolt in a low dimensional space using Rviz
- bolt_hilgendorf: A MoveIt!-based configuration space for single and dual arm motion planning using Bolt
- bolt_ur5: a temporary package leftover from roscon_demo15 for Descartes. Should be merged into bolt\_hilgendorf or removed entirely
- moveit_ompl: based on ``moveit_planners_ompl``, a simplified common interface for moveit and ompl that I've heavily customized
- docker: just contains recipies for auto building docker images on Dockerhub

## Usage

There are currently two main applications for using bolt - 2d and hilgendorf. See the respective folders on how to use them.

## Testing and Linting

To run [roslint](http://wiki.ros.org/roslint), use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/):

    catkin build --no-status --no-deps --this --make-args roslint

To run [catkin lint](https://pypi.python.org/pypi/catkin_lint), use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/):

    catkin lint -W2

There are currently no unit or integration tests for this package. If there were you would use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/):

    catkin run_tests --no-deps --this -i
