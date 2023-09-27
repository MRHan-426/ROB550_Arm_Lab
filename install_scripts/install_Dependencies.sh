#!/bin/bash
#
# Install Script for Armlab
#

#initial update
sudo apt-get update
sudo apt-get upgrade

# dev stuff / code tools
sudo apt-get -y install curl wget build-essential cmake dkms \
    git autoconf automake autotools-dev gdb libglib2.0-dev libgtk2.0-dev \
    libusb-dev libusb-1.0-0-dev freeglut3-dev libboost-dev libgsl-dev \
    net-tools doxygen  

wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | sudo apt-key add -
sudo add-apt-repository -y "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main"
sudo apt-get update
sudo apt install -y code

# Install Qt5 stuff 
sudo apt-get -y install python3-pyqt5
sudo apt -y install pyqt5-dev-tools

#install some python packages
sudo pip install future
sudo pip install modern_robotics

# install ROS2
sudo apt -y install software-properties-common
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
     http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
     | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade
sudo apt -y install ros-humble-desktop
sudo apt -y install ros-dev-tools

# add source setup script 
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# install realsense SDK 2.0
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
sudo apt-get -y install apt-transport-https
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update
sudo apt-get -y install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg

# Install RealSense ROS2 wrapper from ROS servers
sudo apt install -y ros-humble-realsense2-*

#install apriltags
sudo apt-get install -y ros-humble-apriltag-ros
