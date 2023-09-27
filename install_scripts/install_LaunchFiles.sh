#!/bin/bash

# Copy the camera settings
sudo cp config/rs_l515_launch.py /opt/ros/humble/share/realsense2_camera/launch

# Copy tag settings and launch file 
sudo cp config/tags_Standard41h12.yaml /opt/ros/humble/share/apriltag_ros/cfg/

# Indicate completion of a task
echo "Configuration files have been successfully moved."
