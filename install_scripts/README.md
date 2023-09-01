# Here is where armlab starts

First step, we need to setup the environment:

### 1. Install all the dependencies and packages
- If this is your first time setting up, you need to install the necessary dependencies and packages. Open a terminal and navigate to the current folder. Then, run the following command:
```
./install_Dependencies.sh
```
-   Wait until it's complete before proceeding to the next step.

### 2. Install arm related stuff - [Source](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/software_setup.html)
- Copy and run the following command:
```
./install_Interbotix.sh
```
- During the installation, you'll encounter prompts. For prompts related to AprilTag and MATLAB-ROS installation, type no and press Enter.
- ![](/media/interbotix_install.png)
-   Wait until it's complete before proceeding to the next step.


### 3. Move config files
- Copy and run the following command:
```
./install_LaunchFiles.sh
```
- This file is used to move the config files. The configurations are based on the AprilTag family we have and the specific camera model we use.

### 4. Install camera calibration package
- Open a new terminal, then copy and run the following command:
```
./install_Calibration.sh
```

### 5. Set up ROS_DOMAIN_ID
- Copy and run the following command:
```
echo "export ROS_DOMAIN_ID=your_station_number" >> ~/.bashrc
```

- To check if you set it successfully, open a new terminal and run:
```
printenv | grep ROS
```
- where you should see the output has all the environmental variables related to ROS, and you should see `ROS_DOMAIN_ID=the_number_you_set` print out in the terminal

<p align="center">
$\large \color{red}{\textsf{Remember to reboot the computer before using the robot!}}$</p>

After you reboot the laptop, go to [/launch](../launch).

### Q: Why I cannot run the file above?
A: The ".sh" file may not be executable.

![](/media/chmod.png)