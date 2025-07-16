# Use Ubuntu 22.04 as the base image for ROS 2 Humble
FROM ubuntu:22.04

# Set the maintainer label
LABEL maintainer="your-email@example.com"

# Set non-interactive frontend for apt-get
ENV DEBIAN_FRONTEND=noninteractive

# 1. Install base system dependencies and ROS 2 GPG key
RUN apt-get update && \
    apt-get install -y \
    build-essential \
    cmake \
    curl \
    git \
    libpthread-stubs0-dev \
    libnl-3-dev \
    libnl-route-3-dev \
    software-properties-common && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 2. Install ROS 2 Humble and required packages
RUN apt-get update && \
    apt-get install -y \
    ros-humble-ros-base \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-trajectory-controller \
    ros-humble-robot-state-publisher \
    python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/*

# 3. Set up the working directory and copy the entire project
WORKDIR /app
COPY . .

# 4. Build the ROS 2 workspace
# Sourcing ROS 2 is required for the build
#RUN . /opt/ros/humble/setup.sh && \
#    cd ros2_ws && \
#    colcon build --symlink-install

# 5. Set up the entrypoint to source the workspace automatically
# This ensures that any command run in the container has the ROS environment ready
#SHELL ["/bin/bash", "-c"]
#ENTRYPOINT ["source /app/ros2_ws/install/setup.bash && "]

# 6. Define the default command to launch the robot control
# This will be appended to the ENTRYPOINT
#CMD ["ros2", "launch", "eyou_robot_control", "eyou_control.launch.py"]
CMD ["/bin/bash"]