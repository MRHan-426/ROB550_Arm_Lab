# What is this place for?

This directory is intended for storing launch files that can initiate multiple nodes and configure the ROS2 system.

After completing all installation steps, you can begin testing the code using the following commands.

**Table of content**
- [To launch everything](#to-launch-everything)
- [To start single node](#to-start-a-single-node)
    - [Start camera](#start-Realsense2-node)
    - [Start AprilTag detection](#start-AprilTag-Dectection-node)
    - [Start camera calibration](#start-realsense2-node)
    - [Start arm](#start-the-arm-node)

## To launch everything
Open a terminal and navigate to the current folder `/launch`. Run the provided command. Remember that to halt any node, use `ctrl + C` in its respective terminal. Until you stop this node, the terminal will be occupied. If you need to run another command, open a new terminal.
```
./launch_armlab.sh
```
- This one starts all the components we need: camera, apriltag, interbotix_arm.

Then in the new terminal, run the following command:
```
./launch_control_station.sh
```
- This one starts the control station GUI.
- To stop the control station, you just need to close the GUI window.

> Side note: These two files combine multiple single-line commands that we need to start the project, making it simpler to initiate everything from a single location and with fewer open windows.

<p align="center">
$\large \color{red}{\textsf{Do not quit the arm if it's not in the sleep position!!!}}$</p>



### Q: Why I cannot run the file above?
A: The ".sh" file may not be executable.

![](/media/chmod.png)

## To start a single node
#### Start Realsense2 node
```
ros2 launch realsense2_camera rs_l515_launch.py
```

#### Start AprilTag Dectection node 
```
ros2 run apriltag_ros apriltag_node --ros-args \
    -r image_rect:=/camera/color/image_raw \
    -r camera_info:=/camera/color/camera_info \
    --params-file `ros2 pkg prefix apriltag_ros`/share/apriltag_ros/cfg/tags_Standard41h12.yaml
```

#### Start the arm node
```
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=rx200
```
- This command will launch rviz with the virtual robot model, the model would show exactly how the arm moving.
- More can be learned from the official [ROS 2 Quickstart Guide](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/quickstart.html)

#### Start the camera calibration node
```
cd ~/image_pipeline
```
```
source install/setup.bash
```
```
ros2 run camera_calibration cameracalibrator --size 7x9 --square 0.02 --ros-args \
    -r image:=/camera/color/image_raw  \
    -p camera:=/camera
```