# OMPL Bolt

Description: Experience planner based on Thunder

Features:

 - TODO

Developed by [Dave Coleman](http://dav.ee/) at the University of Colorado Boulder

Status:

 * [![Build Status](https://travis-ci.org/davetcoleman/ompl_bolt.svg)](https://travis-ci.org/davetcoleman/ompl_bolt) Travis - Continuous Integration
 * [![Build Status](http://build.ros.org/buildStatus/icon?job=Jsrc_uT__ompl_bolt__ubuntu_trusty__source)](http://build.ros.org/view/Jsrc_uT/job/Jsrc_uT__ompl_bolt__ubuntu_trusty__source/) ROS Buildfarm - Trusty Devel Source Build
 * [![Build Status](http://build.ros.org/buildStatus/icon?job=Jbin_uT64__ompl_bolt__ubuntu_trusty_amd64__binary)](http://build.ros.org/view/Jbin_uT64/job/Jbin_uT64__ompl_bolt__ubuntu_trusty_amd64__binary/) ROS Buildfarm - AMD64 Trusty Debian Build

![](resources/screenshot.png)

## Install

### Ubuntu Debian

> Note: this package has not been released yet

    sudo apt-get install ros-indigo-ompl-bolt

### Build from Source

To build this package, ``git clone`` this repo into a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) and be sure to install necessary dependencies by running the following command in the root of your catkin workspace:

    rosdep install -y --from-paths src --ignore-src --rosdistro indigo

## Code API

> Note: this package has not been released yet

See [Class Reference](http://docs.ros.org/indigo/api/ompl_bolt/html/)

## Usage

TODO

## Developer Notes

There are 6 visualization windows that are each used for multiple different things. Below I try to record their usages

### Window 1

- SparseGraph displayDatabase, addEdge, addVertex

### Window 2

- TaskGraph
- SparseGraph: addStateToRoadmap attempted states
- Connectivity

### Window 3

- Quality criteria: close reps
- Check Add Quality
- Find close representatives

### Window 4

- Astar search?
- Disjoint sets

### Window 5

- Check add path
- Quality criteria

### Window 6

- Start/goal states
- Collision environment
- Projection hack
- Quality criteria
- Remove close vertices
- Interface

### Vertex Colors

 - green:     coverage
 - brown:     connectivity
 - white:     interface
 - pink:      quality
 - purple:    cartesian
 - cyan:      discretized

### Edge Colors

 - orange:    connectivity
 - magenta:    interface
 - red:       quality
 - yellow:   cartesian
 - blue       discretized

## Testing and Linting

To run [roslint](http://wiki.ros.org/roslint), use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/):

    catkin build --no-status --no-deps --this --make-args roslint

To run [catkin lint](https://pypi.python.org/pypi/catkin_lint), use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/):

    catkin lint -W2

There are currently no unit or integration tests for this package. If there were you would use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/):

    catkin run_tests --no-deps --this -i

## Contribute

Please send PRs for new helper functions, fixes, etc!
