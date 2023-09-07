#!/bin/bash
gnome-terminal --tab --title='Realsense' -- bash -c 'ros2 launch realsense2_camera rs_l515_launch.py; read -p "Press Enter to exit"'
sleep 2
gnome-terminal --tab --title='Apriltag' -- bash -c 'ros2 run apriltag_ros apriltag_node --ros-args -r image_rect:=/camera/color/image_raw -r camera_info:=/camera/color/camera_info --params-file `ros2 pkg prefix apriltag_ros`/share/apriltag_ros/cfg/tags_Standard41h12.yaml; read -p "Press Enter to exit"'
sleep 2
gnome-terminal --tab --title='XSArm Control' -- bash -c 'ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=rx200; read -p "Press Enter to exit"'
