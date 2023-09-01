 #!/bin/bash

# install ROS2 Camera Calibration package
# https://navigation.ros.org/tutorials/docs/camera_calibration.html

sudo apt install ros-humble-camera-calibration-parsers
sudo apt install ros-humble-camera-info-manager
sudo apt install ros-humble-launch-testing-ament-cmake

cd ~
git clone -b humble https://github.com/ros-perception/image_pipeline.git
cd ~/image_pipeline
colcon build --symlink-install
