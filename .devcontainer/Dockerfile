FROM osrf/ros:humble-desktop-full

ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build tools
    build-essential \
    cmake \
    git \
    wget \
    curl \
    sudo \
    # ROS2 additional packages
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-moveit \
    ros-humble-moveit-visual-tools \
    ros-humble-moveit-ros-visualization \
    ros-humble-moveit-planners \
    ros-humble-moveit-simple-controller-manager \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-controller-manager \
    ros-humble-joint-trajectory-controller \
    ros-humble-joint-state-broadcaster \
    ros-humble-effort-controllers \
    ros-humble-velocity-controllers \
    ros-humble-camera-info-manager \
    ros-humble-image-transport \
    ros-humble-cv-bridge \
    ros-humble-vision-opencv \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-rviz2 \
    # Gazebo
    gazebo \
    libgazebo-dev \
    # Python packages
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    # Graphics / headless rendering
    mesa-utils \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Install Python scientific packages
RUN pip3 install --no-cache-dir \
    casadi \
    numpy \
    matplotlib \
    opencv-python-headless \
    scipy \
    transforms3d \
    modern-robotics

# Create non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Set up ROS2 environment for the user
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc \
    && echo "source /opt/ros/humble/setup.zsh 2>/dev/null || true" >> /home/$USERNAME/.zshrc \
    && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/$USERNAME/.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /home/$USERNAME/.zshrc \
    && echo '[ -f /workspaces/*/install/setup.bash ] && source /workspaces/*/install/setup.bash 2>/dev/null || true' >> /home/$USERNAME/.bashrc \
    && echo '[ -f /workspaces/*/install/setup.zsh ] && source /workspaces/*/install/setup.zsh 2>/dev/null || true' >> /home/$USERNAME/.zshrc

USER $USERNAME
WORKDIR /home/$USERNAME
