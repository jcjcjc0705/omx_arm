ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base-noble AS base
LABEL org.opencontainers.image.source="https://github.com/jcjcjc0705/omx_arm"

ARG ROS_DISTRO
ARG DEBIAN_FRONTEND=noninteractive

ENV ROS_DISTRO=${ROS_DISTRO} \
    ROS2_WS=/ws

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-ros2-control \
        ros-${ROS_DISTRO}-ros2-controllers \
        ros-${ROS_DISTRO}-dynamixel-sdk \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*

FROM base AS dev

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=ubuntu

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        python3-rosdep \
        ros-${ROS_DISTRO}-rviz2 \
        ros-${ROS_DISTRO}-joint-state-publisher-gui \
        git vim less gdb \
    && rm -rf /var/lib/apt/lists/*

RUN echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME}

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER ${USERNAME}
RUN rosdep update --rosdistro ${ROS_DISTRO}

WORKDIR ${ROS2_WS}
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
