# davetcoleman/bolt:bolt-kinetic-source

FROM moveit/moveit_docker:moveit-kinetic-ci
MAINTAINER Dave Coleman dave@dav.ee

ENV CATKIN_WS=/root/ws_moveit
WORKDIR $CATKIN_WS/src

# delete old source code, download bolt source
RUN cd .. && rm -rf src/ && mkdir src && cd src && \
    wstool init && \
    wstool merge https://raw.githubusercontent.com/davetcoleman/bolt/${ROS_DISTRO}-devel/bolt.${ROS_DISTRO}.rosinstall && \
    wstool update

RUN apt-get update && \
    apt-get install -y wget && \
    wget https://raw.githubusercontent.com/ros-gbp/ompl-release/debian/${ROS_DISTRO}/xenial/ompl/package.xml -O ompl/package.xml && \
    # disable packages that do not have proper dependencies
    touch moveit/moveit_planners/ompl/CATKIN_IGNORE && \
    # touch robotiq/robotiq_action_server/CATKIN_IGNORE && \
    # touch robotiq/robotiq_c_model_control/CATKIN_IGNORE && \
    # touch robotiq/robotiq_ethercat/CATKIN_IGNORE && \
    # touch robotiq/robotiq_s_model_control/CATKIN_IGNORE && \
    # touch robotiq/robotiq_joint_state_publisher/CATKIN_IGNORE && \
    # touch universal_robot/ur_gazebo/CATKIN_IGNORE && \
    apt-get remove -y ros-${ROS_DISTRO}-ompl && \
    rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false && \
    apt-get remove -y ros-${ROS_DISTRO}-ompl && \
    rm -rf /var/lib/apt/lists/*
