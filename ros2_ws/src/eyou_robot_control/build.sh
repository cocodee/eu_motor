#!/bin/bash

# This script builds the eyou_robot_control package.
#
# ==============================================================================
# IMPORTANT: You MUST edit the line below to provide the correct path to your
# ROS 2 installation's setup.bash file.
# ==============================================================================
ROS2_SETUP_PATH="/opt/ros/humble/setup.bash" # <-- EDIT THIS PATH

# --- Do not edit below this line ---

set -e

# Check if the ROS 2 setup file exists
if [ ! -f "$ROS2_SETUP_PATH" ]; then
    echo "Error: ROS 2 setup file not found at '$ROS2_SETUP_PATH'"
    echo "Please edit the ROS2_SETUP_PATH variable in this script."
    exit 1
fi

echo "Sourcing ROS 2 environment from: $ROS2_SETUP_PATH"
source "$ROS2_SETUP_PATH"

# Navigate to the workspace root directory from the script's location
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
WORKSPACE_DIR="$SCRIPT_DIR/../.." # Go up two levels from src/eyou_robot_control

echo "Navigating to workspace: $WORKSPACE_DIR"
cd "$WORKSPACE_DIR"

echo "Running colcon build for eyou_robot_control..."
colcon build --packages-select eyou_robot_control

echo ""
echo "Build complete."
echo "To use the package, source the new setup file in your terminal:"
echo "source $WORKSPACE_DIR/install/setup.bash"
