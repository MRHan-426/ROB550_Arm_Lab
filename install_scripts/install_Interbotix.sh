 #!/bin/bash
#install interbotix-robot-arm

source /opt/ros/humble/setup.bash
curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_manipulators/main/interbotix_ros_xsarms/install/amd64/xsarm_amd64_install.sh' > xsarm_amd64_install.sh
chmod +x xsarm_amd64_install.sh
sudo rosdep init
rosdep update
./xsarm_amd64_install.sh -d humble 
echo "source ~/interbotix_ws/install/setup.bash" >> ~/.bashrc

