ARG ROS_DISTRO=humble
FROM ros:$ROS_DISTRO AS base
ARG DEBIAN_FRONTEND=noninteractive

ARG CONTAINER_USER="cuser"
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a user in the container to operate as unprivileged
RUN groupadd --gid $USER_GID $CONTAINER_USER \
    && useradd --uid $USER_UID --gid $USER_GID -m $CONTAINER_USER \
    && mkdir -p /home/$CONTAINER_USER
RUN apt-get update \
    && apt-get install -y sudo \
    && rm -rf /var/lib/apt/lists/* \
    && echo $CONTAINER_USER ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$CONTAINER_USER \
    && chmod 0440 /etc/sudoers.d/$CONTAINER_USER

SHELL ["/bin/bash", "--login", "-c"]

# Install dependencies
RUN apt-get update && apt-get install -y \
    curl \
    git \
    libclang-dev \
    tmux \
    python3-pip \
    ssh \
    xterm \
    tree \
    ament-cmake \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install -y ros-$ROS_DISTRO-gazebo-ros2-control \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-robot-localization \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-joint-state-broadcaster \
    ros-$ROS_DISTRO-joint-trajectory-controller \
    ros-$ROS_DISTRO-rclcpp \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-teleop-twist-keyboard \
    ros-$ROS_DISTRO-gazebo-ros2-control \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-robot-localization \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-velodyne \
    ros-$ROS_DISTRO-velodyne-gazebo-plugins \
    ros-$ROS_DISTRO-velodyne-description \
    && rm -rf /var/lib/apt/lists/*


COPY ./ /home/${CONTAINER_USER}/ros_ws/src/ros2_rust_workshop
RUN vcs import /home/${CONTAINER_USER}/ros_ws/src < /home/${CONTAINER_USER}/ros_ws/src/ros2_rust_workshop/ros2_rust_humble.repos && \
    cd /home/${CONTAINER_USER}/ros_ws/src && git clone --recursive https://github.com/anujjain-dev/unitree-go2-ros2.git -b humble



# Install Rust and the cargo-ament-build plugin
RUN chown -R ${CONTAINER_USER}:${CONTAINER_USER} /home/${CONTAINER_USER}/ros_ws
USER $CONTAINER_USER
WORKDIR /home/${CONTAINER_USER}
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- --default-toolchain 1.75.0 -y
ENV PATH="/home/${CONTAINER_USER}/.cargo/bin:$PATH"
RUN cargo install cargo-ament-build

RUN pip install --upgrade pytest

# Install the colcon-cargo and colcon-ros-cargo plugins
RUN pip install git+https://github.com/colcon/colcon-cargo.git git+https://github.com/colcon/colcon-ros-cargo.git
WORKDIR /home/${CONTAINER_USER}/ros_ws/src

