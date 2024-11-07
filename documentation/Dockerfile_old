# old not working docker file for reference
FROM arm64v8/ros:humble-ros-base
# alt: humble-ros-base-jammy
# osrf/ros:humble-desktop-full DOES NOT WORK FOR linux/arm64
# same for ghcr.io/sloretz/ros:humble-desktop & ros:humble-ros-base-jammy
# Ubuntu 22.04 Jammy Jellyfish

WORKDIR /

# Install necessary packages
RUN apt-get update
RUN apt-get upgrade -y
RUN apt-get install -y \
    sudo \
    nano \
    curl \
    git \
    x11-apps \
    ros-dev-tools \
    python3-pip \
    # Install Nav2 dependencies
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    # Install Turtlebot3 dependencies
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-dynamixel-sdk \
    ros-humble-turtlebot3-msgs \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot4-desktop \
    ros-humble-turtlebot4-simulator

# install ros2 packages - so that it's same as humble-desktop
RUN apt-get update
RUN apt-get install -y --no-install-recommends \
    ros-humble-desktop=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

# install (ignition) gazebo fortress
# reference: https://gazebosim.org/docs/fortress/install_ubuntu/
RUN sudo apt-get install -y lsb-release gnupg
RUN sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN sudo apt-get update
RUN sudo apt-get install -y ignition-fortress

RUN sudo apt-get install -y software-properties-common

# install dependencies for nav2 -- ERROR!!
RUN sudo add-apt-repository -y ppa:openrobotics/gazebo11-non-amd64
RUN sudo apt update
# RUN apt-get install -y \
#     ros-humble-gazebo-ros \
#     ros-humble-turtlebot3-gazebo

# # install dependencies for imu
RUN apt-get install -y \
    ament-cmake

SHELL ["/bin/bash", "-c"]

# Create workspace directory
RUN mkdir -p ~/ros2_ws/src
# # Clone source code into workspace
WORKDIR /ros2_ws
COPY /src /ros2_ws/src

# setup colcon_cd
WORKDIR /
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
RUN echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc

# other setup
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "export QT_QPA_PLATFORM=xcb" >> ~/.bashrc
RUN echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc

# for nav2
RUN echo "source /usr/share/gazebo/setup.bash" >> ~/.bashrc
RUN echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
# RUN echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models" >> ~/.bashrc

# source the bashrc
RUN bash -c "source ~/.bashrc"

# install for ros2_control
WORKDIR /ros2_ws
RUN sudo apt-get install -y ros-humble-ros2-control
RUN vcs import --input https://raw.githubusercontent.com/ros-controls/ros2_control_ci/master/ros_controls.$ROS_DISTRO.repos src
RUN sudo rosdep fix-permissions \ 
    && rosdep update --rosdistro=$ROS_DISTRO
RUN sudo apt-get update


# Change to the working directory
WORKDIR /ros2_ws

# Command to run on container start for creating this image
#
# for Nav2 dependencies:
# sudo nano /etc/apt/sources.list
# (paste: deb http://packages.osrfoundation.org/gazebo/ubuntu-nightly jammy main)
# wget  http://packages.osrfoundation.org/gazebo.key
# sudo apt-key add gazebo.key
# sudo apt-get update
# sudo apt-get install -y gazebo
# [if no these steps: missing extra pkg - ros-humble-gazebo-dev] <-- required for colcon build (gazebo_ros2_control)
#
# must have these line before colcon build
# RUN rosdep install --from-paths src --ignore-src -r -y
# RUN . /opt/ros/${ROS_DISTRO}/setup.sh
# WORKDIR /ros2_ws
# RUN colcon build --symlink-install (times 3)
# 
# RUN colcon build --packages-select serial_imu
# RUN bash -c "source /ros2_ws/install/setup.bash"