#!/bin/bash
set -e

echo "=== Post-create setup starting ==="

# Source ROS2
source /opt/ros/humble/setup.bash

# Initialize rosdep if not already done
sudo rosdep init 2>/dev/null || true
rosdep update --rosdistro=humble

# Install workspace dependencies
cd /workspaces/$(basename $PWD)
rosdep install --from-paths src --ignore-src -r -y 2>/dev/null || true

# Build the workspace
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
echo "source $(pwd)/install/setup.bash" >> ~/.bashrc
echo "source $(pwd)/install/setup.zsh 2>/dev/null || true" >> ~/.zshrc

echo "=== Post-create setup complete ==="
echo "Access the VNC desktop at: http://localhost:6080"
echo "Run 'ros2 launch space_arm_simulation full_simulation.launch.py' to start"
