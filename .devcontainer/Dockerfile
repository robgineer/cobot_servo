# syntax=docker/dockerfile:1
FROM ros:jazzy

# update apt for further downloads
RUN apt-get update

# install required utilities
RUN apt-get install -y apt-utils software-properties-common x11-apps wget vim curl
# install dev. packages
RUN apt-get install -y git build-essential python-is-python3 python3-tqdm python3-ipykernel python3-venv
# get zsh and zimfw
RUN apt-get install -y zsh && curl -fsSL https://raw.githubusercontent.com/zimfw/install/master/install.zsh | zsh

## set locales for ROS2 Jazzy
RUN apt-get install -y locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# install ROS2 control
RUN apt-get update
RUN apt-get install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers

# install Moveit2
RUN apt-get install -y ros-jazzy-moveit ros-jazzy-moveit-py ros-jazzy-moveit-servo ros-jazzy-control-msgs ros-jazzy-moveit-visual-tools ros-jazzy-moveit-ros-perception ros-jazzy-moveit-ros-control-interface

## install xpra (with latest version) ##
RUN wget -O "/usr/share/keyrings/xpra.asc" https://xpra.org/xpra.asc \
    && wget -P /etc/apt/sources.list.d/ https://raw.githubusercontent.com/Xpra-org/xpra/master/packaging/repos/noble/xpra.sources \
    && apt-get update \
    && apt-get install -y xpra

## setup venv
RUN python3 -m venv --system-site-packages /opt/ros_venv
RUN /opt/ros_venv/bin/pip install --upgrade pip setuptools wheel colcon-common-extensions scipy scikit-learn mediapipe

#################################################
## port current user to container if required ##
#################################################
ARG USERNAME=*IAmNotAUser*
ARG USER_ID=1000
ARG GROUP_ID=1000

RUN if [ "$USERNAME" != "*IAmNotAUser*" ] ; then \
    # create the user
    groupadd --gid $GROUP_ID $USERNAME && \
    useradd --uid $USER_ID --gid $GROUP_ID -m $USERNAME && \
    apt-get update && apt-get install -y sudo && \
    # enable sudo with no password (there is no need for that but its a nice to have)
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    # create directories required for xpra logging
    mkdir -p /run/user/$USER_ID && chown $USERNAME /run/user/$USER_ID ; \
    fi

