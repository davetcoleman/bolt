# davetcoleman/curie:curie-jade

FROM davetcoleman/curie:curie-jade-source
MAINTAINER Dave Coleman dave@dav.ee

ENV CATKIN_WS=/root/ws_moveit

# build repo
WORKDIR $CATKIN_WS
ENV TERM xterm
ENV PYTHONIOENCODING UTF-8
RUN source /ros_entrypoint.sh && \
    catkin build --no-status -DCMAKE_BUILD_TYPE=Release

# RUN source devel/setup.bash && \
#     roslaunch curie_demos hilgendorf_benchmark.launch && \
#     cat ~/ros/ompl_storage/benchmark_bolt_right_arm_0.250000_database.benchmark
