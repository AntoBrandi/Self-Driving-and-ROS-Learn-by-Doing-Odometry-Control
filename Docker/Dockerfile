FROM osrf/ros:noetic-desktop-full

ARG USER=user
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
      ros-noetic-ros-controllers \
      ros-noetic-gazebo-ros-control \
      ros-noetic-actionlib-tools \
      ros-noetic-joint-state-publisher-gui \
      ros-noetic-turtlesim \
      ros-noetic-robot-localization \
      ros-noetic-joy-teleop \
      ros-noetic-joy \
      terminator \
      python3-pip \
    && rm -rf /var/lib/apt/lists/*
      
WORKDIR /home/${USER}