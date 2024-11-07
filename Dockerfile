FROM arm64v8/ros:humble-ros-base

WORKDIR /

# Install necessary packages
RUN apt-get update
RUN apt-get upgrade -y
RUN apt-get install -y \
    python3-pip \
    nano

# install ros2 packages - so that it's same as humble-desktop
RUN apt-get update
RUN apt-get install -y --no-install-recommends \
    ros-humble-desktop=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*
    
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

# source the bashrc
RUN bash -c "source ~/.bashrc"

# Change to the working directory
WORKDIR /ros2_ws

RUN apt-get update && apt-get install -y ros-humble-imu-tools

RUN rosdep install --from-paths src --ignore-src -r -y
RUN . /opt/ros/${ROS_DISTRO}/setup.sh

############
# Pending changes 

# For Built version:
    # RUN colcon build --symlink-install
    # RUN colcon build --packages-select serial_imu
    # RUN bash -c "source /ros2_ws/install/setup.bash"

# next test items:
# rviz2 test for imu visualisation -> check if imu's data is really working
# explore use of tf2 & stuff for getting the position