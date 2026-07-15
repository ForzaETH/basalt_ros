ARG DISTRO=humble
FROM ros:$DISTRO-ros-base
SHELL [ "/bin/bash", "-c" ]
ARG USERNAME=foo
ARG USER_UID=1
ARG USER_GID=1
ARG ROS_WS=/ws
# install ros packages
ARG DEBIAN_FRONTEND=noninteractive
ADD .docker_utils/${ROS_DISTRO}_req.txt /.install_utils/linux_req.txt
RUN apt-get update && xargs apt-get install -y < /.install_utils/linux_req.txt

# install python packages
# RUN --mount=type=cache,target=/root/.cache/pip pip3 install -r /.install_utils/requirements.txt

ADD .docker_utils/setup_user.sh /.install_utils/user.sh
RUN chmod +x /.install_utils/user.sh && /.install_utils/user.sh "$USERNAME" "$USER_UID" "$USER_GID"

RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}

COPY . src/basalt_ros/

RUN vcs import --recursive < src/basalt_ros/basalt_ros.repos
RUN vcs import --recursive < src/basalt_wrapper/basalt_wrapper.repos

# Uncomment the following lines to build your own workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.sh && colcon build --merge-install --symlink-install --cmake-args -DCMAKE_CXX_FLAGS="-w" -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON

RUN chown -R ${USERNAME} ${ROS_WS}
COPY .docker_utils/entrypoint.sh /entrypoint.sh
# COPY .docker_utils/healthcheck.sh /healthcheck.sh
# RUN chmod +x /healthcheck.sh
RUN chmod +x /entrypoint.sh
RUN chown ${USERNAME} /entrypoint.sh
# RUN chown ${USERNAME} /healthcheck.sh
USER ${USERNAME}
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
# HEALTHCHECK --interval=10s --timeout=5s --start-period=5s --retries=2 CMD /healthcheck.sh
ENTRYPOINT ["/entrypoint.sh"]