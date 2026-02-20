#!/bin/bash
# Install Python dependencies for ROS2 bag manipulation

echo "Installing Python dependencies for ROS2 bag processing..."

# Install rosbag2 Python packages
pip install rosbag2-py

# If using ROS2, these should already be available, but just in case:
pip install rclpy

echo ""
echo "Installation complete!"
echo ""
echo "To use the script:"
echo "  python3 create_baro_bag.py [input_bag] [output_bag] [frequency]"
echo ""
echo "Example:"
echo "  python3 create_baro_bag.py ../V1_02_medium Test_Baro_Zeros_BadCam 20"
