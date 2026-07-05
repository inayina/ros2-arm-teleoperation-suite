# ROS 2 Jazzy image for ros2-arm-teleoperation-suite.
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy \
    ROS2_WS_ROOT=/ws

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-controller-manager \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-forward-command-controller \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-moveit-servo \
    ros-jazzy-moveit-kinematics \
    ros-jazzy-diagnostic-aggregator \
    ros-jazzy-rqt-robot-monitor \
    ros-jazzy-rqt-runtime-monitor \
    ros-jazzy-realtime-tools \
    can-utils \
    libglfw3-dev \
    libxxf86vm-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

# Install python dependencies
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --break-system-packages --no-cache-dir -r /tmp/requirements.txt

# Copy source code and build
COPY . /ws/src/ros2-arm-teleoperation-suite

RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install

# Copy entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
