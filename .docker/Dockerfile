ARG ROS_DISTRO=humble
FROM ros:$ROS_DISTRO AS base
ARG DEBIAN_FRONTEND=noninteractive

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

RUN sudo apt-get update
RUN sudo apt-get install -y ros-$ROS_DISTRO-gazebo-ros2-control
RUN sudo apt-get install -y ros-$ROS_DISTRO-xacro
RUN sudo apt-get install -y ros-$ROS_DISTRO-robot-localization
RUN sudo apt-get install -y ros-$ROS_DISTRO-rviz2
RUN sudo apt-get install -y ros-$ROS_DISTRO-ros2-control
RUN sudo apt-get install -y ros-$ROS_DISTRO-joint-state-broadcaster
RUN sudo apt-get install -y ros-$ROS_DISTRO-joint-trajectory-controller
RUN sudo apt-get install -y ros-$ROS_DISTRO-rclcpp
RUN sudo apt-get install -y ros-$ROS_DISTRO-nav2-bringup


# Install Rust and the cargo-ament-build plugin
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- --default-toolchain 1.74.0 -y
ENV PATH=/root/.cargo/bin:$PATH
RUN cargo install cargo-ament-build

RUN pip install --upgrade pytest 

# Install the colcon-cargo and colcon-ros-cargo plugins
RUN pip install git+https://github.com/colcon/colcon-cargo.git git+https://github.com/colcon/colcon-ros-cargo.git
