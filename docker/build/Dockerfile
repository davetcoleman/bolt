# davetcoleman/bolt:bolt-jade

FROM davetcoleman/bolt:bolt-jade-source
MAINTAINER Dave Coleman dave@dav.ee

ENV CATKIN_WS=/root/ws_moveit

# build repo
WORKDIR $CATKIN_WS
ENV TERM xterm
ENV PYTHONIOENCODING UTF-8
RUN source /ros_entrypoint.sh && \
    catkin build --no-status -DCMAKE_BUILD_TYPE=Release
