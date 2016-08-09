# BOLT

A new planner based on SPARS2.

Developed by [Dave Coleman](http://dav.ee/) at the University of Colorado Boulder in collaboration with ROS Industrial, Southwest Research Institute, and the National Institute of Standards and Technology.

Status:

 * [![Travis Build Status](https://travis-ci.org/davetcoleman/bolt.svg)](https://travis-ci.org/davetcoleman/bolt) Travis - Continuous Integration

![](bolt_hilgendorf/resources/screenshot.png)

### Build from Source

Currently tested for ROS Indigo. To build this package, create a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

    wstool init .
    wstool merge https://raw.githubusercontent.com/davetcoleman/bolt/jade-devel/bolt.rosinstall
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

## Testing and Linting

To run [roslint](http://wiki.ros.org/roslint), use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/):

    catkin build --no-status --no-deps --this --make-args roslint

To run [catkin lint](https://pypi.python.org/pypi/catkin_lint), use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/):

    catkin lint -W2

There are currently no unit or integration tests for this package. If there were you would use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/):

    catkin run_tests --no-deps --this -i
