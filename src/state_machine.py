"""!
The state machine that implements the logic.
"""
from PyQt5.QtCore import QThread, Qt, pyqtSignal, pyqtSlot, QTimer
import time
import numpy as np
import rclpy
import kinematics

from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from apriltag_msgs.msg import *
from block_detection import *

class StateMachine():
    """!
    @brief      This class describes a state machine.

                Add states and state functions to this class to implement all of the required logic for the armlab
    """

    def __init__(self, rxarm, camera):
        """!
        @brief      Constructs a new instance.

        @param      rxarm   The rxarm
        @param      planner  The planner
        @param      camera   The camera
        """
        self.rxarm = rxarm
        self.camera = camera
        self.status_message = "State: Idle"
        self.current_state = "idle"
        self.next_state = "idle"
        self.remembered_waypoint = []
        self.intrinsic_matrix = self.camera.intrinsic_matrix
        self.extrinsic_matrix = self.camera.extrinsic_matrix
        self.dh_params = kinematics.parse_dh_param_file("../config/rx200_dh.csv")
        self.is_detect = False
        self.waypoints = [[-np.pi/2,            -0.5,         -0.3,              0.0,         0.0, 1],
                          [0.75*-np.pi/2,        0.5,          0.3,         -np.pi/3,     np.pi/2, 1],
                          [0.5*-np.pi/2,        -0.5,         -0.3,        np.pi / 2,         0.0, 1],
                          [0.25*-np.pi/2,        0.5,          0.3,         -np.pi/3,     np.pi/2, 1],
                          [0.0,                  0.0,          0.0,              0.0,         0.0, 1],
                          [0.25*np.pi/2,        -0.5,         -0.3,              0.0,     np.pi/2, 1],
                          [0.5*np.pi/2,          0.5,          0.3,         -np.pi/3,         0.0, 1],
                          [0.75*np.pi/2,        -0.5,         -0.3,              0.0,     np.pi/2, 1],
                          [np.pi/2,              0.5,          0.3,         -np.pi/3,         0.0, 1],
                          [0.0,                  0.0,          0.0,              0.0,         0.0, 1]]
        self.detection = detection(self.camera)
        
        self.node = rclpy.create_node('JB_node')
        self.replay_pub = self.node.create_publisher(
            Int32,
            'JB_replay',
            10)
        self.calibrate_pub = self.node.create_publisher(
            Int32,
            'JB_calibrate',
            10)

    def initialize_rxarm(self, task = False):
        """!
        @brief      Initializes the rxarm.
        """
        self.current_state = "initialize_rxarm"
        self.status_message = "RXArm Initialized!"
        self.remembered_waypoint = []
        if not self.rxarm.initialize(task = task):
            print('Failed to initialize the rxarm')
            self.status_message = "State: Failed to initialize the rxarm!"
            time.sleep(5)
        self.next_state = "idle"

    def set_next_state(self, state):
        """!
        @brief      Sets the next state.

            This is in a different thread than run so we do nothing here and let run handle it on the next iteration.

        @param      state  a string representing the next state.
        """
        self.next_state = state

    def run(self):
        """!
        @brief      Run the logic for the next state

                    This is run in its own thread.

                    Add states and funcitons as needed.
        """
        if self.next_state == "initialize_rxarm":
            self.initialize_rxarm()

        if self.next_state == "idle":
            self.idle()

        if self.next_state == "estop":
            self.estop()

        if self.next_state == "execute":
            self.execute()

        if self.next_state == "calibrate":
            self.calibrate()

        if self.next_state == "detect":
            self.detect()

        if self.next_state == "record":
            self.record()

        if self.next_state == "replay":
            self.replay()

        if self.next_state == "open_gripper":
            self.open_gripper()

        if self.next_state == "close_gripper":
            self.close_gripper()

        if self.next_state == "grab":
            self.grab()
        
        if self.next_state == "pick_n_sort":
            self.pick_n_sort()
        
        if self.next_state == "pick_n_stack":
            self.pick_n_stack()

        if self.next_state == "line_em_up":
            self.line_em_up()

        if self.next_state == "stack_em_high":
            self.stack_em_high()
        
        if self.next_state == "to_the_sky":
            self.to_the_sky()
        
        if self.next_state == "motion_test":
            self.kinematics_motion_test()

    """Functions run for each state"""

    def manual(self):
        """!
        @brief      Manually control the rxarm
        """
        self.status_message = "State: Manual - Use sliders to control arm"
        self.current_state = "manual"

    def idle(self):
        """!
        @brief      Do nothing
        """
        self.status_message = "State: Idle - Waiting for input"
        self.current_state = "idle"

    def estop(self):
        """!
        @brief      Emergency stop disable torque.
        """
        self.status_message = "EMERGENCY STOP - Check rxarm and restart program"
        self.current_state = "estop"
        self.rxarm.disable_torque()

    def execute(self):
        """!
        @brief      Go through all waypoints
        TODO: Implement this function to execute a waypoint plan
              Make sure you respect estop signal
        """
        self.status_message = "State: Execute - Executing motion plan"
        self.current_state = "execute"
        for n in self.waypoints:
            print("reached")
            self.rxarm.set_positions(n[:-1])
            time.sleep(3)
        self.next_state = "idle"

    def record(self):
        """!
        @brief      Record positions of the arm, one click for one position.
        """
        self.status_message = "State: record current position"
        self.current_state = "record"

        if self.remembered_waypoint == []:
            self.rxarm.sleep()
            time.sleep(3)
            self.rxarm.disable_torque()
            self.remembered_waypoint.append(self.waypoints[0])
        else:
            curr_pos = self.rxarm.get_positions()
            curr_pos = np.append(curr_pos,self.remembered_waypoint[-1][-1])
            print("======================================================")
            print("WayPoint Recorded:")
            print(curr_pos)
            print("======================================================")
            self.remembered_waypoint.append(curr_pos)
        self.next_state = "idle"

    def close_gripper(self) :
        """!
        @brief      close the gripper of the arm.
        """
        self.status_message = "State: close gripper"
        self.current_state = "close_gripper"
        if self.remembered_waypoint != []:
            self.remembered_waypoint[-1][-1] = 0
            self.rxarm.enable_torque()
            time.sleep(0.2)
            self.rxarm.gripper.grasp()
            time.sleep(0.5)
            self.rxarm.disable_torque()
        else:
            self.rxarm.gripper.grasp()
            time.sleep(0.5)
        self.next_state = "idle"

    def open_gripper(self):
        """!
        @brief      open the gripper of the arm.
        """
        self.status_message = "State: open gripper"
        self.current_state = "open_gripper"
        if self.remembered_waypoint == []:
            self.rxarm.gripper.release()
            time.sleep(1)
        else:
            self.remembered_waypoint[-1][-1] = 1
            self.rxarm.enable_torque()
            time.sleep(0.2)
            self.rxarm.gripper.release()
            time.sleep(1)
            self.rxarm.disable_torque()
        self.next_state = "idle"
        
    def replay(self):
        """!
        @brief      Replay those recorded positions.
        """
        self.status_message = "State: replay recorded position "
        self.current_state = "replay"

        msg = Int32()
        msg.data = 1
        self.replay_pub.publish(msg)

        self.rxarm.enable_torque()
        time.sleep(0.2)
        self.rxarm.gripper.release()
        time.sleep(0.2)

        for n in self.remembered_waypoint[1:]:
            self.rxarm.set_positions(n[:-1])
            time.sleep(1.5)
            if not n[-1]:
                self.rxarm.gripper.grasp()
                time.sleep(0.3)
            else:
                self.rxarm.gripper.release()
                time.sleep(0.3)

        msg.data = 0
        self.replay_pub.publish(msg)
        self.next_state = "idle"

    def calibrate(self):
        """!
        @brief      Gets the user input to perform the calibration
        
        @rostopic   'JB_calibrate', Int32, when data is set to 1, start calibrate.
        """
        self.current_state = "calibrate"
        self.next_state = "idle"
        msg = Int32()
        msg.data = 1
        self.calibrate_pub.publish(msg)
        msg.data = 0
        self.calibrate_pub.publish(msg)
        self.status_message = "Calibration - Completed Calibration"

    def detect(self):
        """
        @brief Perform block detection.

        This method toggles the block detection process on and off. When block detection
        is enabled, it runs the detection process and updates the video frame with the
        detected blocks. When disabled, it sets the status message to indicate that
        detection has ended.

        @return None
        """
        self.current_state = "detect"
        self.next_state = "idle"

        self.is_detect = not self.is_detect
        if self.is_detect:
            self.detection.run(only_blocks = False)
            output = self.detection.drawblock(show_boundary = False)
            self.camera.VideoFrame = output
            self.status_message = "Detect color, position, orientation of blocks"
        else:
            self.status_message = "Detect End"
    
    def kinematics_motion_test(self):
        self.current_state = "motion_test"
        self.next_state = "idle"

        # Define target points as a list of positions
        target_points = [
            [0, 275, 0, np.pi / 2],
            [-250, 225, 0, np.pi / 2],
            [-300, 125, 0, np.pi / 2],
            [-300, 25, 0, np.pi / 2],
            [-250, -75, 0, np.pi / 2],
            [-250, -125, 0, np.pi / 2]
        ]

        for i, target_pos in enumerate(target_points):
            can_reach, joint_angles = kinematics.IK_geometric(target_pos)

            if can_reach:
                print(f"Move to point {i + 1}")
                self.rxarm.set_positions(joint_angles)
                time.sleep(2)
                curr_pos = self.rxarm.get_positions()
                print("Current pos is: ", curr_pos)
                time.sleep(3)

                # Adjust joint angles if needed
                joint_angles[1] = -joint_angles[1]
                joint_angles[2] = -joint_angles[2]
                joint_angles[3] = -joint_angles[3]

                # Calculate T_DH if needed
                T_DH = kinematics.FK_dh(dh_params=self.dh_params, joint_angles=joint_angles, link=5)
                print("T_DH is: ", T_DH)
            else:
                print(f"Cannot reach point {i + 1}")

        print("Motion Test Done!")

    def calMoveTime(self, target_joint,slowmode = False):
        """
        @brief Calculate the movement time and acceleration time for reaching a target joint position.

        This method calculates the time required to move the robot arm to a target joint position.
        It also calculates the acceleration time based on whether slow mode is enabled.

        @param target_joint: A numpy array representing the target joint positions.
        @param slowmode: If True, slow mode is enabled, which affects the velocity and acceleration profiles.
                        Defaults to False.

        @return tuple: A tuple containing the total movement time and acceleration time.
        """
        displacement = np.array(target_joint) - self.rxarm.get_positions()
        max_joint_diff = np.max(np.abs(displacement))

        if np.abs(displacement[0])>np.pi/4 or np.abs(displacement[1])>np.pi/4:
            slowmode = True

        if slowmode:
            angular_v = np.ones(displacement.shape) * (np.pi / 5)
            angular_v[0] = np.pi / 5
            angular_v[1] = np.pi / 5
            angular_v[4] = np.pi / 3
            angular_t = np.abs(displacement) / angular_v
            move_time = np.max(angular_t)
            if move_time < 0.4:
                move_time = 0.4
            ac_time = move_time / 2
            return move_time, ac_time
        else:
            angular_v = np.ones(displacement.shape) * (np.pi / 4)
            angular_v[0] = np.pi / 4
            angular_v[1] = np.pi / 4
            angular_v[4] = np.pi / 2.5
            angular_t = np.abs(displacement) / angular_v
            move_time = np.max(angular_t)
            if move_time < 0.4:
                move_time = 0.4
            ac_time = move_time / 3
            return move_time, ac_time
    
    def get_grab_point(self):
        """!
        @brief      Judge if it is a valid click point

                    return (x,y,z) of block center in the world frame
        """
        self.camera.new_click = False
        while not self.camera.new_click:
            time.sleep(0.01)

        for block in self.detection.blocks:
            if block.inArea(self.camera.last_click):
                return self.camera.transformFromImageToWorldFrame((block.center[1], block.center[0])),block.orientation
                
        return None, None

    def grab(self):
        """!
        @brief     use IK to grab a block at given position
        """
        self.current_state = "grab"
        self.next_state = "idle"
        if not self.is_detect:
            print("Please click detect first, then click grab again!")
            return None

        while not self.detection.blocks:
            print("There is no blocks in the workspace!!")
            time.sleep(1)

        print("Please click a block in the workspace")
        grab_point, ee_orientation = self.get_grab_point()

        while grab_point is None:
            print("There is no block, Please click again!")
            grab_point, ee_orientation = self.get_grab_point()

        # click_point = self.camera.last_click_worldframe
        print("Grab task start!")

        self.grab_or_put_down_a_block(click_point=grab_point, is_grab=True, ee_orientation=ee_orientation)
        print("Successfully grab the block, please click to put it down!")

        self.camera.new_click = False
        while not self.camera.new_click:
            time.sleep(0.01)

        put_down_point = self.camera.last_click_worldframe
        print("put_down_point", put_down_point)
        self.grab_or_put_down_a_block(click_point=put_down_point, is_grab=False)
        print("Successfully put down the block, mission complete!")

    def grab_or_put_down_a_block(self, click_point, is_grab, ee_orientation = 0):
        """
        @brief Perform grabbing or putting down a block operation.

        This method controls the robot arm to either grab or put down a block at the specified
        click point with the given end-effector orientation.

        @param click_point: A numpy array representing the target click point [x, y, z].
        @param is_grab: A boolean indicating whether to grab (True) or put down (False) the block.
        @param ee_orientation: The desired end-effector orientation in radians. Defaults to 0.

        @return None
        """
        if is_grab:
            pose = [click_point[0], click_point[1], click_point[2] - 10, np.pi/3]
        else:
            pose = [click_point[0], click_point[1], click_point[2] + 30, np.pi/3]

        if is_grab:
            self.pre_position = pose.copy()
            self.pre_position[2] += 100
            success1, joint_pos1 = kinematics.IK_geometric(self.pre_position,block_ori=ee_orientation)
            success2, joint_pos2 = kinematics.IK_geometric(pose,block_ori=ee_orientation)
            if success1:
                self.rxarm.set_positions(joint_pos1)
                time.sleep(3)
                if success2:
                    self.rxarm.set_positions(joint_pos2)
                    time.sleep(3)
                else:
                    print("Point cannot access!!")
            else:
                print("Pre point cannot access!!")
            self.rxarm.gripper.grasp()
            time.sleep(0.3)
        else:
            # current_joint_angles = self.rxarm.get_positions()
            pose3 = pose.copy()
            pose2 = pose.copy()
            pose2[2] += 100
            success1, joint_pos1 = kinematics.IK_geometric(self.pre_position)
            success2, joint_pos2 = kinematics.IK_geometric(pose2)
            success3, joint_pos3 = kinematics.IK_geometric(pose3)
            if success1:
                self.rxarm.set_positions(joint_pos1)
                time.sleep(3)
                if success2:
                    self.rxarm.set_positions(joint_pos2)
                    time.sleep(3)
                    if success3:
                        self.rxarm.set_positions(joint_pos3)
                        time.sleep(3)
                        self.rxarm.gripper.release()
                        time.sleep(0.5)
                        self.rxarm.set_positions(joint_pos2)
                        time.sleep(3)
                    else:
                        print("Cannot reach pose3 !")
                        self.initialize_rxarm()
                        time.sleep(1)
                else:
                    print("Cannot reach pose2 !")
                    self.initialize_rxarm()
                    time.sleep(1)
            else:
                print("Cannot reach pose1 !")
                self.initialize_rxarm()
                time.sleep(1)

    def safe_motion(self, target_joint_angles, slow_mode=False):
        """
        @brief Perform safe motion planning for reaching a target joint position.

        This method plans a safe trajectory to reach the target joint angles, and it may
        interpolate intermediate points if the distance is too long. It takes into account
        whether slow mode is enabled for motion planning.

        @param target_joint_angles: A numpy array representing the target joint angles.
        @param slow_mode: If True, slow mode is enabled, which affects the velocity and acceleration profiles.
                        Defaults to False.

        @return None
        """
        current_joint_angles = self.rxarm.get_positions()
        displacement = np.array(target_joint_angles) - current_joint_angles
        first_4_elements = displacement[:4]
        max_joint_diff = np.max(np.abs(first_4_elements))
        n = int(max_joint_diff // (np.pi / 15))
        print("middle points number is: ", n)
        # Add n points in the trajectory if the distance is too long
        if n > 0:
            displacement_unit = displacement / n
            for i in range(n):
                middle_point = current_joint_angles + displacement_unit * i
                move_time, ac_time = self.calMoveTime(middle_point, slowmode=slow_mode)
                self.rxarm.arm.set_joint_positions(middle_point,
                                                moving_time=move_time,
                                                accel_time=ac_time,
                                                blocking=False)
                time.sleep(0.08)
        move_time, ac_time = self.calMoveTime(target_joint_angles, slowmode=slow_mode)
        self.rxarm.arm.set_joint_positions(target_joint_angles,
                                        moving_time=move_time,
                                        accel_time=ac_time,
                                        blocking=True)
        time.sleep(0.1)

    def safe_pos(self):
        """
        @brief Move the robot to a safe position and update its status
        
        This method moves the robot to a predefined safe position and updates its status accordingly.
        
        @param self: The object instance.
        @return None
        """
        self.status_message = "State: Returning to safe position"
        self.current_state = "safe_pos"
        safe_pos = [0,0,0,0,0]
        move_time,ac_time = self.calMoveTime([0,0,0,0,0])
        self.rxarm.arm.set_joint_positions([0,0,0,0,0],
                                           moving_time = move_time, 
                                           accel_time = ac_time,
                                           blocking = True)
        print("Safe Pos: Successfully Reach Safe Pos")
        time.sleep(0.1)
        self.next_state = "idle"

    def compute_ee_world_pos(self):
        """
        @brief Compute the end effector world position using DH forward kinematics.

        This method calculates the world position of the end effector by applying the
        Denavit-Hartenberg (DH) forward kinematics using the current joint angles.

        @return list: A list containing the [x, y, z] world position of the end effector.
        """
        current_joint_angles = self.rxarm.get_positions()
        T_DH = kinematics.FK_dh(dh_params=self.dh_params, joint_angles=current_joint_angles, link=5)
        world_pos = T_DH[0:3,3]
        print(world_pos)
        world_pos.tolist()
        return world_pos

    def sky_walker(self, camera_clean = False, task5 = False, first_to_left = False, post_to_left = False):
        """!
        @brief      raise the arm up to 1: make camera vision clear
                                        2: prevent sweeping other blocks
                                        3: pre-pose for to-the-sky
        """
        current_joint_angles = self.rxarm.get_positions()

        if task5:
            if first_to_left:
                target_joint = [0,-np.pi/12,-np.pi/5,0,0]
                target_joint2 = [-np.pi/4,-np.pi/12,-np.pi/5,0,0]
            elif post_to_left:
                target_joint = [-np.pi/4,-np.pi/12,-np.pi/5,0,0]
                target_joint2 = [0,-np.pi/12,-np.pi/5,0,0]
            else:
                target_joint = [-np.pi/4,-np.pi/12,-np.pi/5,0,0]
            
        elif camera_clean:
            self.rxarm.gripper.release()
            target_joint = [0,-np.pi/4,-np.pi/6 ,0,0]
        else:
            theta0 = current_joint_angles[0]
            target_joint = [theta0,-np.pi/5,-np.pi/8,np.pi/6,0]
        
        if task5:
            if first_to_left or post_to_left:
                self.safe_motion(target_joint)
                self.safe_motion(target_joint2)
            else:
                self.safe_motion(target_joint)
        else:
            self.safe_motion(target_joint)
        
        time.sleep(0.5)
        print("Auto Place: Reach Pos1")

    def throw_away(self):
        """!
        @brief      throw something away to the negtive y plane 
        """
        current_joint_angles = self.rxarm.get_positions()
        theta0 = current_joint_angles[0]
        target_joint1 = [theta0,-np.pi/8,0,np.pi/6,0]
        target_joint2 = [np.pi * (3/2),0,0,np.pi/6,0]
        self.safe_motion(target_joint1)
        self.safe_motion(target_joint2)
        time.sleep(0.5)
        self.rxarm.gripper.release()
        time.sleep(1)
        self.initialize_rxarm()
        time.sleep(0.5)    
        print("Throw Away: Complete")

    def pose_compute(self, pos, block_ori, isvertical = False):
        """!
        @brief      compute end effector pose [x,y,z,phi] given x,y,z 
                    and orientation of the block, for general picking situation 
        """
        pos2 = pos[:]
        x = pos2[0]
        y = pos2[1]
        z = pos2[2]
        orientaion = block_ori
        phi = np.pi/2
        can_Pick = False

        can_Pick,joint_angles = kinematics.IK_geometric([x,y,z,phi],block_ori=orientaion,isVertical_Pick=isvertical)
        if can_Pick:
            print("Pose Compute: Success")
            return True,joint_angles
        else:
            for i in range(2):
                x = x * 0.99
                y = y * 0.99
                phi = phi - np.pi*(1.0/30.0)
                can_Pick,joint_angles = kinematics.IK_geometric([x,y,z,phi],block_ori=orientaion,isVertical_Pick=isvertical)
                print("Pose Compute: adjust")
                if can_Pick:
                    print("Pose Compute: Success")
                    return True, joint_angles
            
            correction_counter = 0
            while not can_Pick:
                phi = phi - np.pi*(1.0/90.0)
                can_Pick,joint_angles = kinematics.IK_geometric([x,y,z,phi],block_ori=orientaion,isVertical_Pick=isvertical)
                correction_counter = correction_counter + 1
                print("Pose Compute: adjust")
                if can_Pick:
                    print("Pose Compute: Success")
                    return True,joint_angles
                if correction_counter > 5:
                    print("Pose Compute: Failure Can not even reach corrected Pose")
                    print("Pose Compute: The point that cannot reach is: ", [x,y,z])
                    return False,[0,0,0,0,0]
    
    def loose_pose_compute(self,pos,block_ori,isvertical = False):
        """!
        @brief      compute pose for those pre_positions or not important positions
                    so there is no strict requirement for phi
        """
        pos2 = pos[:]
        x = pos2[0]
        y = pos2[1]
        z = pos2[2]
        phi = np.pi/2
        orientation = block_ori
        can_reach = False
        can_reach,joint_angles = kinematics.IK_geometric([x,y,z,phi],block_ori=orientation,isVertical_Pick=isvertical)
        if can_reach:
            print("Loose Pose Compute: Success")
            return True,joint_angles
        else:
            for i in range (30):
                x = x * 0.99
                y = y * 0.99
                z = z - 0.1
                phi = phi - np.pi * (1.0/90.0)
                can_reach,joint_angles = kinematics.IK_geometric([x,y,z,phi],block_ori=orientation,isVertical_Pick=isvertical)
                print("Loose Pose Compute: adjust")
                if can_reach:
                    print("Loose Pose Compute: Success")
                    return True,joint_angles
                
            print("Loose Pose Compute: Failure")
            return False,[0,0,0,0,0]
                
    def horizontal_pose_compute(self,pos,block_ori = 0):
        """
        @brief Compute the horizontal pose for a given position.

        This method calculates the joint angles required to achieve a horizontal pose at the
        specified position [x, y, z] with a given block orientation (phi).

        @param pos: A list representing the target position [x, y, z].
        @param block_ori: The desired block orientation (phi) in radians. Defaults to 0.

        @return tuple: A tuple containing a boolean indicating whether the pose is reachable,
                    and the joint angles if reachable, or [0, 0, 0, 0, 0] if not reachable.
        """
        phi = 0
        can_reach = False
        pos2 = pos[:]
        x = pos2[0]
        y = pos2[1]
        z = pos2[2]

        can_reach,joint_angles = kinematics.IK_geometric([x,y,z,phi])
        if can_reach:
            print("Horizontal Pose Compute: Success")
            return True,joint_angles
        else:
            print("Horizontal Pose Compute: Failure")
            return False, [0,0,0,0,0]
       
    def horizontal_loose_pose_compute(self,pos,block_ori = 0):
        """
        @brief Compute the horizontal pose for a given position.

        This method calculates the joint angles required to achieve a horizontal pose at the
        specified position [x, y, z] with a given block orientation (phi).

        @param pos: A list representing the target position [x, y, z].
        @param block_ori: The desired block orientation (phi) in radians. Defaults to 0.

        @return tuple: A tuple containing a boolean indicating whether the pose is reachable,
                    and the joint angles if reachable, or [0, 0, 0, 0, 0] if not reachable.
        """
        phi = 0
        can_reach = False
        pos2 = pos[:]
        x = pos2[0]
        y = pos2[1]
        z = pos2[2]
        can_reach,joint_angles = kinematics.IK_geometric([x,y,z,phi])
        if can_reach:
            print("Horizontal Loose Pose Compute: Success")
            return True,joint_angles
        else:
            for i in range(10):
                x = x * 0.99
                y = y * 0.99
                phi = phi + np.pi/180
                print("Horizontal Loose Pose Compute: Adjust")
                can_reach,joint_angles = kinematics.IK_geometric([x,y,z,phi])
                if can_reach:
                    print("Horizontal Loose Pose Compute: Success")
                    return True,joint_angles
            print("Horizontal Loose Pose Compute: Failure")
            return False, [0,0,0,0,0]

    """Logic Functions for tasks"""

    def auto_pick(self, target_pos, block_ori, depth = 0, isbig = False, isStack = False, saveTime = False, vertical = False):
        """
        @brief Automatically go to a position and pick the block there.

        This method automatically plans and executes the motion to pick a block at the specified target
        position with the given block orientation and parameters such as depth, block size, stacking,
        and saving time.

        @param target_pos: A list representing the target position [x, y, z].
        @param block_ori: The desired block orientation in radians.
        @param depth: The depth parameter (optional). Defaults to 0.
        @param isbig: A boolean indicating whether the block is big (True) or not (False). Defaults to False.
        @param isStack: A boolean indicating whether stacking is enabled (True) or not (False). Defaults to False.
        @param saveTime: A boolean indicating whether to save time during motion (True) or not (False). Defaults to False.
        @param vertical: A boolean indicating whether the motion is vertical (True) or not (False). Defaults to False.

        @return None
        """
        orientation = block_ori
        
        # pos1 is pre_post position, pos2 is pick position, pos3 is post_pick position
        target_pos = kinematics.Target_Pos_Compensation(list(target_pos))
        pos1 = list(target_pos[:])
        pos2 = list(target_pos[:])
        pos3 = list(target_pos[:])

        if isbig:
            pick_offset = 38 # compensation for block height
        else:
            pick_offset = 35
            
        if isStack and not isbig and depth > 55:
            pick_offset = 26
        elif isStack and not isbig and depth < 55:
            pick_offset = 22
        elif isStack and isbig:
            pick_offset = 30

        pick_height = 100 # place gripper above block

        pos2[2] = pos2[2] - pick_offset
        pos1[2] = pos1[2] + pick_height
        pos3[2] = pos3[2] + pick_height

        reachable1, joint_angles1 = self.loose_pose_compute(tuple(pos1),block_ori=orientation,isvertical=vertical)
        reachable2, joint_angles2 = self.pose_compute(pos = tuple(pos2),block_ori=orientation,isvertical=vertical)
        current_joint_angles = self.rxarm.get_positions()
        time.sleep(0.5)

        if reachable1 and reachable2:
            self.safe_motion(joint_angles1)
            # go to the pre-picking point
            move_time,ac_time = self.calMoveTime(joint_angles1)
            self.rxarm.arm.set_joint_positions(joint_angles1,
                                           moving_time = move_time, 
                                           accel_time = ac_time,
                                           blocking = False)
            print("Auto Pick: Reach Pos1")
            time.sleep(0.1)
            if saveTime:
                # go the the picking point not using dichotomy
                move_time,ac_time = self.calMoveTime(joint_angles2)
                self.rxarm.arm.set_joint_positions(joint_angles2,
                                            moving_time = move_time, 
                                            accel_time = ac_time,
                                            blocking = True)
                print("Auto Pick: Reach Pos2")
                time.sleep(0.1)
            else:
                # go to the picking point using dichotomy 
                displacement = np.array(joint_angles2) - np.array(joint_angles1)
                displacement_unit =  displacement
                temp_joint = np.array(joint_angles1)
                last_effort = self.rxarm.get_efforts()       
                for i in range(4):
                    displacement_unit = displacement_unit / 2
                    temp_joint = temp_joint + displacement_unit
                    move_time,ac_time = self.calMoveTime(temp_joint)
                    self.rxarm.arm.set_joint_positions(temp_joint.tolist(),
                                            moving_time = move_time, 
                                            accel_time = ac_time,
                                            blocking = True)
                    time.sleep(0.1)
                    effort = self.rxarm.get_efforts()
                    effort_difference = (effort - last_effort)[1:3]
                    last_effort = effort
                    effort_diff_norm = np.linalg.norm(effort_difference)
                    if i > 2 and effort_diff_norm > 50:
                        print("Auto Pick: Effort difference is: ", effort_diff_norm)
                        print("Auto Pick: Reach Pos2 in advance")
                        break
            print("Auto Pick: Reach Pos2")
            # close the gripper and pick the block
            self.rxarm.gripper.grasp()
            time.sleep(1)
            self.safe_motion(joint_angles1)
            print("Auto Pick: Reach Pos3")
            print("Auto Pick: Success")
            time.sleep(0.1)
        else:
            print("Auto Pick: Unreachable Position!")
            time.sleep(0.1)

    def auto_place(self, target_pos, target_orientation, isbig = False, save_time = False, isTask5 = False, vertical = False):
        """
        @brief Automatically go to a position and place the block there.

        This method automatically plans and executes the motion to place a block at the specified target
        position with the given block orientation and parameters such as block size, saving time, and
        whether it is a Task 5 operation.

        @param target_pos: A list representing the target position [x, y, z].
        @param target_orientation: The desired block orientation in radians.
        @param isbig: A boolean indicating whether the block is big (True) or not (False). Defaults to False.
        @param save_time: A boolean indicating whether to save time during motion (True) or not (False). Defaults to False.
        @param vertical: A boolean indicating whether the motion is vertical (True) or not (False). Defaults to False.
        @param isTask5: A boolean indicating whether it is a Task 5 operation (True) or not (False). Defaults to False.

        @return None
        """
        orientation = target_orientation
        target_pos = kinematics.Target_Pos_Compensation(list(target_pos))
        isVertical = vertical

        # pos1 is pre_post position, pos2 is pick position, pos3 is post_pick position
        pos1 = list(target_pos[:])
        pos2 = list(target_pos[:])
        pos3 = list(target_pos[:])

        # compensation for block height
        if isbig:
            place_offset = 0
        else:
            place_offset = -10

        place_height = 70 # place gripper above block
        pos2[2] = pos2[2] + place_offset
        pos1[2] = pos1[2] + place_height
        pos3[2] = pos3[2] + place_height

        reachable1, joint_angles1 = self.loose_pose_compute(tuple(pos1),block_ori=orientation,isvertical=vertical)
        reachable2, joint_angles2 = self.pose_compute(pos = tuple(pos2),block_ori=orientation,isvertical=vertical)
        current_joint_angles = self.rxarm.get_positions()
        time.sleep(0.5)

        if reachable1 and reachable2:
            self.safe_motion(joint_angles1)
            time.sleep(1)
            displacement = np.array(joint_angles2) - np.array(joint_angles1)
            displacement_unit =  displacement
            temp_joint = np.array(joint_angles1)
            last_effort = self.rxarm.get_efforts()
            if save_time:
                # not using dichotomy
                move_time,ac_time = self.calMoveTime(joint_angles2)
                self.rxarm.arm.set_joint_positions(joint_angles2,
                                            moving_time = move_time, 
                                            accel_time = ac_time,
                                            blocking = True)
                print("Auto Place: Reach Pos2")
                time.sleep(0.1)
            else: 
                # Use dichotomy to place the block accurately
                for i in range(10):
                    displacement_unit = displacement_unit / 2
                    temp_joint = temp_joint + displacement_unit
                    move_time,ac_time = self.calMoveTime(temp_joint)
                    self.rxarm.arm.set_joint_positions(temp_joint.tolist(),
                                               moving_time = move_time, 
                                               accel_time = ac_time,
                                               blocking = True)
                    time.sleep(0.1)
                    effort = self.rxarm.get_efforts()
                    effort_difference = (effort - last_effort)[1:3]
                    last_effort = effort
                    effort_diff_norm = np.linalg.norm(effort_difference)
                    if i > 2 and effort_diff_norm > 50:
                        print("Auto Place: Effort difference is: ", effort_diff_norm)
                        print("Auto Place: Reach Pos2 in advance")
                        break
                print("Auto Place: Reach Pos2")
            time.sleep(0.3)
            self.rxarm.gripper.release()
            if isTask5:
                time.sleep(0.5)
                self.rxarm.gripper.grasp()
                time.sleep(1)
                self.rxarm.gripper.release()
                time.sleep(1)
            self.safe_motion(joint_angles1)
            print("Auto Place: Reach Pos3")
            print("Auto Place: Success")
        else:
            print("Auto Place: Unreachable Position!")
            time.sleep(0.2)

    def horizontal_auto_place(self, target_pos, superhigh = False, isTask5 = False):
        """
        @brief Automatically go to a position and horizontally place the block there.

        This method automatically plans and executes the horizontal motion to place a block at the specified target
        position with the given parameters such as block size, superhigh placement, and whether it is a Task 5 operation.

        @param target_pos: A list representing the target position [x, y, z].
        @param superhigh: A boolean indicating whether to perform superhigh placement (True) or not (False). Defaults to False.
        @param isTask5: A boolean indicating whether it is a Task 5 operation (True) or not (False). Defaults to False.

        @return None
        """
        target_pos = kinematics.Target_Pos_Compensation(list(target_pos))
        # pos1 is pre_post position, pos2 is pick position, pos3 is post_pick position
        pos1 = list(target_pos[:])
        pos2 = list(target_pos[:])
        pos3 = list(target_pos[:])
        # compensation for block height
        place_offset = 20
        place_height = 90 # place gripper above block
        if not superhigh:
            pos2[2] = pos2[2] + place_offset
            pos1[2] = pos1[2] + place_height
            pos3[2] = pos3[2] + place_height
        else:
            pos2[2] = pos2[2] + place_offset
            pos1[2] = pos1[2] + 50
            pos1[1] = pos1[1] - 40
            pos3[2] = pos3[2] + 50
            pos3[1] = pos3[1] - 40

        reachable1, joint_angles1 = self.horizontal_loose_pose_compute(tuple(pos1),block_ori=0)
        reachable2, joint_angles2 = self.horizontal_pose_compute(pos = tuple(pos2),block_ori=0)
        time.sleep(0.5)

        current_joint_angles = self.rxarm.get_positions()
        if reachable1 and reachable2:
            self.safe_motion(joint_angles1)
            displacement = np.array(joint_angles2) - np.array(joint_angles1)
            displacement_unit =  displacement
            temp_joint = np.array(joint_angles1)
            last_effort = self.rxarm.get_efforts()
            # Use dichotomy to place the block accurately
            for i in range(10):
                displacement_unit = displacement_unit / 2
                temp_joint = temp_joint + displacement_unit
                move_time,ac_time = self.calMoveTime(temp_joint)
                self.rxarm.arm.set_joint_positions(temp_joint.tolist(),
                                            moving_time = move_time,
                                            accel_time = ac_time,
                                            blocking = True)
                time.sleep(0.1)
                effort = self.rxarm.get_efforts()
                effort_difference = (effort - last_effort)[1:3]
                last_effort = effort
                effort_diff_norm = np.linalg.norm(effort_difference)
           
                if i > 2 and effort_diff_norm > 40:
                    print("Auto Place: Effort difference is: ", effort_diff_norm)
                    print("Auto Place: Reach Pos2 in advance")
                    break
               
            print("Auto Place: Reach Pos2")
            self.rxarm.gripper.release()
            time.sleep(1)

            if isTask5:
                time.sleep(0.5)
                self.rxarm.gripper.grasp()
                time.sleep(1)
                self.rxarm.gripper.release()
                time.sleep(1)

            self.safe_motion(joint_angles1)
            print("Auto Place: Reach Pos3")
            time.sleep(0.1)
            print("Auto Place: Success")
        else:
            print("Auto Place: Unreachable Position!")
            time.sleep(0.2)

    def auto_change_position(self,current_pos):
        """
        @brief Automatically go to a position and horizontally place the block there.

        This method automatically plans and executes the horizontal motion to place a block at the specified target
        position with the given parameters such as block size, superhigh placement, and whether it is a Task 5 operation.

        @param target_pos: A list representing the target position [x, y, z].
        @param superhigh: A boolean indicating whether to perform superhigh placement (True) or not (False). Defaults to False.
        @param isTask5: A boolean indicating whether it is a Task 5 operation (True) or not (False). Defaults to False.

        @return None
        """
        target_pos = current_pos.copy()
        xy_distance = np.sqrt(np.square(current_pos[0])+np.square(current_pos[1]))
        if xy_distance > 350:
            target_pos[0] = target_pos[0]*3/4
            target_pos[1] = target_pos[1]*3/4
            target_pos[2] = 50
        else:
            target_pos[0] = target_pos[0]*(8.0/7.0)
            target_pos[1] = target_pos[1]*(8.0/7.0)
            target_pos[2] = 50
        can_reach,target_joint_angles = self.loose_pose_compute(target_pos,block_ori=0)

        if can_reach:
            time.sleep(1)
            self.safe_motion(target_joint_angles)

            time.sleep(0.2)
            final_pos = self.compute_ee_world_pos()
            print("Auto Change Position: reach bomb position: ", final_pos)
            
            self.rxarm.gripper.release()
            time.sleep(0.5)
            print("Auto Change Position: Success")
        else:
            print("Auto Change Position: Failure")

    # Event 1:Pick'n sort!
    def pick_n_sort(self,istask3 = False):
        if not istask3:
            self.current_state = "pick_n_sort"
            self.next_state = "idle"
            print("##################### Pick 'n sort Start #####################")
               
        self.block_number_counter = 1
        self.small_counter, self.big_counter = 0,0
        self.small_x , self.big_x = -160,160
        self.small_y,self.big_y = 25,25
        if istask3:
            block_offset = 80
            safe_area_y = 0
            boundary = self.camera.boundary[4:6]
        else:
            boundary = self.camera.boundary[2:4]
            block_offset = 70
            safe_area_y = 0
        # 1st time deal with 2 floor blocks, 2nd time deal with general blocks
        while True:
            # Detect blocks in the plane
            self.detection.run(only_blocks = True, boundary = boundary)
            time.sleep(1)
            while not self.detection.blocks:
                print("There is no blocks in the workspace!!")
                time.sleep(1)
            
            # all_block_in_neg_plane = True
            valid_blocks = []
            for block in self.detection.blocks:
                if block.type == "small" or block.type == "big":
                    valid_blocks.append(block)

            if valid_blocks == []:
                break

            # Initialize place positions - x coordinates           
            self.initialize_rxarm()
            time.sleep(2)

            # for block in self.detection.blocks:
            for block in valid_blocks:
                if block.depth < 150: 
                    print("--------------------- start a block:No.",self.block_number_counter,"---------------------------")
                    print("block_depth", block.depth)

                    # if it's stack 1 high, pick and place normally
                    if block.type == "small":
                        if block.world_xyz[0] >= 0 or block.world_xyz[1] >= safe_area_y:
                            print("=========== Small ",block.world_xyz," =============")
                            if block.stack == False:
                                block.depth = 25
                            else:
                                block.depth = block.depth
                            self.auto_pick(target_pos= block.world_xyz, block_ori = block.orientation, isbig=False, isStack = block.stack, depth=block.depth)
                            time.sleep(0.2)
                            print("gripper position:", self.rxarm.get_gripper_position())

                            if self.rxarm.get_gripper_position() > 0.02:
                                print("Actually it is a big block")
                                if self.big_counter % 3 == 0:
                                    self.big_x = 160
                                    if self.big_counter > 1:
                                        self.big_y -= block_offset
                                else:
                                    self.big_x += block_offset

                                self.auto_place(target_pos=[self.big_x,self.big_y,0],target_orientation = 0,isbig = True,save_time=True)
                                self.big_counter +=1
                                self.block_number_counter += 1
                            else:
                            # compute the place position   
                                if self.small_counter % 2 == 0:
                                    self.small_x = -160
                                    if self.small_counter > 1:
                                        self.small_y -= block_offset
                                else:
                                    self.small_x -= block_offset

                                self.auto_place(target_pos=[self.small_x,self.small_y,0],target_orientation = 0,isbig=False,save_time=True)
                                self.small_counter += 1
                                self.block_number_counter += 1
                    # Move big blocks in left plane to right plane
                    elif block.type == "big":
                        if block.world_xyz[0] <= 0 or block.world_xyz[1] >= safe_area_y:
                            print("=========== Big ",block.world_xyz," =============")
                            if block.stack == False:
                                block.depth = 38
                            else:
                                block.depth = block.depth
                            self.auto_pick(target_pos=block.world_xyz,block_ori = block.orientation ,isbig=True ,isStack = block.stack, depth = block.depth)
                            time.sleep(0.2)
                            print("gripper position:", self.rxarm.get_gripper_position())

                            if self.rxarm.get_gripper_position() < 0.02:
                                print("Actually it is a small block")
                                # compute the place position   
                                if self.small_counter % 2 == 0:
                                    self.small_x = -160
                                    if self.small_counter > 1:
                                        self.small_y -= block_offset
                                else:
                                    self.small_x -= block_offset

                                self.auto_place(target_pos=[self.small_x,self.small_y,0],target_orientation = 0,isbig=False,save_time=True)
                                self.small_counter += 1
                                self.block_number_counter += 1
                            else:
                                # compute the place position
                                if self.big_counter % 3 == 0:
                                    self.big_x = 160
                                    if self.big_counter > 1:
                                        self.big_y -= block_offset
                                else:
                                    self.big_x += block_offset

                                self.auto_place(target_pos=[self.big_x,self.big_y,0],target_orientation = 0,isbig = True,save_time=True)
                                self.big_counter +=1
                                self.block_number_counter += 1
                    time.sleep(0.2)
                    print("--------------------- end a block ---------------------------")

        self.initialize_rxarm()
        time.sleep(0.5)
        self.rxarm.arm.go_to_sleep_pose(moving_time = 1.5,
                          accel_time=0.5,
                          blocking=True)
        print("##################### Pick 'n sort finished #####################")    

    # Event 2 :Pick'n stack!
    def pick_n_stack(self):
        self.current_state = "pick_n_stack"
        self.next_state = "idle"
        print("##################### Pick 'n stack Start #####################")
               
        self.block_number_counter = 1
        self.small_counter,self.big_counter = 0,0
        self.small_x , self.big_x = -250,250
        self.small_y,  self.big_y = -25,-25
        self.small_z,self.big_z = 0,0

        while True:
            # Detect blocks in the plane
            self.detection.run(only_blocks = True, boundary = self.camera.boundary[2:4])
            time.sleep(1)

            while not self.detection.blocks:
                print("There is no blocks in the workspace!!")
                time.sleep(1)
            
            valid_blocks = []
            for block in self.detection.blocks:
                if block.type == "small" or block.type == "big":
                    valid_blocks.append(block)

            if valid_blocks == []:
                break

            valid_blocks.sort(key=lambda block: block.stack)
            has_big = False
            has_stack = False
            self.small_blocks_count = 0
            for block in valid_blocks:
                if block.type == "big":
                    has_big = True
                else:
                    self.small_blocks_count += 1
                if block.stack == True:
                    has_stack = True

            # Initialize place positions - x coordinates
            self.initialize_rxarm()
            time.sleep(2)

            if has_stack:
                for block in valid_blocks:
                    if block.stack == True:
                        block.depth = block.depth
                    elif block.type == "big":
                        block.depth = 38
                    else:
                        block.depth = 25
                        
                    if self.small_blocks_count >= 3:
                        self.task2_pick_and_put_blocks(block)
                        continue
                    elif self.small_blocks_count < 3 and block.type == "big":
                        self.task2_pick_and_put_blocks(block, throw = True)
                        continue
                    else:
                        if block.stack:
                            # self.small_blocks_count < 3 and block.type == "small"
                            print("============= Break Stack ===============")
                            self.auto_pick(target_pos=self.block.world_xyz,block_ori = self.block.orientation ,isbig=False, isStack=block.stack,saveTime=False)

                            time.sleep(0.2)
                            current_position = self.compute_ee_world_pos()
                            self.auto_change_position(current_pos=current_position)
                            print("=========== Break Stack Comptele =============")
                        else:
                            continue
                    
                self.initialize_rxarm()
                time.sleep(0.5)
                self.rxarm.arm.go_to_sleep_pose(moving_time = 1.5,
                                                accel_time=0.5,
                                                blocking=True)
                continue
            
            if has_big:
                for block in valid_blocks:
                    self.block.world_xyz, self.block.orientation  = self.camera.transformFromImageToWorldFrame((block.center[1], block.center[0])),block.orientation

                    if block.type == "big":
                        block.depth = 38
                    else:
                        block.depth = 25
                    
                    if block.type == "big":
                        self.task2_pick_and_put_blocks(block, throw=True)
                        continue
                    else:
                        continue
                self.initialize_rxarm()
                time.sleep(0.5)
                self.rxarm.arm.go_to_sleep_pose(moving_time = 1.5,
                        accel_time=0.5,
                        blocking=True)
                continue
            
            for block in valid_blocks:
                self.block.world_xyz, self.block.orientation  = self.camera.transformFromImageToWorldFrame((block.center[1], block.center[0])),block.orientation 
                
                if block.type == "big":
                    block.depth = 38
                else:
                    block.depth = 25

                self.task2_pick_and_put_blocks(block, throw=False)

                time.sleep(0.2)
            
            self.initialize_rxarm()
            time.sleep(0.5)
            self.rxarm.arm.go_to_sleep_pose(moving_time = 1.5,
                                accel_time=0.5,
                                blocking=True)
        print("##################### Pick 'n stack finished #####################") 

    def task2_pick_and_put_blocks(self, block, throw = False)-> bool:
        if block.type == "big":
            isbig = True
        else:
            isbig = False

        self.auto_pick(target_pos=block.world_xyz,block_ori = block.orientation, isbig=isbig, saveTime=False)
        time.sleep(0.2)
        
        if self.rxarm.get_gripper_position() < 0.02 and block.type == "big":
            print("Actually it is a small block")
            block.type = "small"
            if throw:
                self.rxarm.gripper.release()
                time.sleep(0.5)
                return
        elif self.rxarm.get_gripper_position() > 0.02 and block.type == "small":
            print("Actually it is a big block")
            block.type = "big"
        
        if block.type == "small":
            if self.small_counter >= 3:
                self.auto_place(target_pos=[self.big_x,self.big_y,self.big_z],target_orientation = 0,isbig=False,save_time=True)
                self.big_counter += 1
                self.block_number_counter += 1
                self.big_z += 25
            else:
                self.auto_place(target_pos=[self.small_x,self.small_y,self.small_z],target_orientation = 0,isbig=False,save_time=True)
                self.small_counter += 1
                self.block_number_counter += 1
                self.small_z += 25
        else:
            if self.big_counter >= 3:
                self.auto_place(target_pos=[self.small_x,self.small_y,self.small_z],target_orientation = 0,isbig=True,save_time=True)
                self.small_counter += 1
                self.block_number_counter += 1
                self.small_z += 38     
            else:
                self.auto_place(target_pos=[self.big_x,self.big_y,self.big_z],target_orientation = 0,isbig=True,save_time=True)
                self.big_counter += 1
                self.block_number_counter += 1
                self.big_z += 38
        time.sleep(0.2)
        print("=============Finish Stacking a "+ block.type + "block===============")

    # Event 3 :Line 'em up!
    def line_em_up(self):
        self.current_state = "line_em_up"
        self.next_state = "idle"
        print("##################### Line 'em up Start #####################")

        print("----------------------Step1: Start to Line Up---------------------")
        color_order = {"red": 0, "orange": 1, "yellow": 2, "green": 3, "blue": 4, "purple": 5}
        # [x,y]
        color_place_big = {"red": [350, -25, 0], "orange": [305, -25, 0],
                            "yellow": [260, -25, 0], "green": [215, -25, 0],
                            "blue": [170, -25, 0], "purple": [125, -25, 0]}

        color_place_small = {"red": [-350,-25, 0], "orange": [-305,-25, 0],
                            "yellow": [-260,-25, 0], "green": [-215,-25, 0],
                            "blue": [-170,-25, 0], "purple": [-125,-25, 0]}

        color_line_big = {"red": False, "orange": False,
                        "yellow": False, "green": False,
                        "blue": False, "purple": False}
        color_line_small = {"red": False, "orange": False,
                        "yellow": False, "green": False,
                        "blue": False, "purple": False}
        while(True):
            if all(color_line_big.values()) and all(color_line_small.values()):
                print("--------------------Mission Complete---------------------")
                break
           
            self.sky_walker(camera_clean = True)
            self.detection.run(only_blocks = False, boundary = self.camera.boundary[8:10])
            sorted_blocks = sorted(self.detection.blocks, key=lambda x: color_order.get(x.color, len(color_order)))

            if sorted_blocks == []:
                print("--------------------There is no more blocks---------------------")
                print("--------------------Detect Again---------------------")
                continue

            for block in sorted_blocks:
                if block.type == "big":
                    block.depth = 38
                    isbig = True
                else:
                    block.depth = 25
                    isbig = False
                if block.stack == True:
                    block.depth = block.depth

                self.auto_pick(target_pos=block.world_xyz, depth= block.depth,
                                block_ori = block.orientation ,
                                isbig=isbig, isStack=block.stack, saveTime=False)
                # time.sleep(0.5)
                self.sky_walker()

                if self.rxarm.get_gripper_position() > 0.02:
                    # Big block
                    if not color_line_big[block.color]:
                        target_pos = color_place_big[block.color]

                        self.auto_place(target_pos= target_pos, target_orientation = 0,isbig = True, save_time=False, vertical = True)
                        # time.sleep(0.5)
                        
                        color_line_big[block.color] = True
                        self.sky_walker()
                        # time.sleep(0.5)

                    else:
                        print("----------------------We fuck up, something wrong---------------------")
                        self.rxarm.gripper.release()
                        time.sleep(0.5)
                        self.sky_walker()
                        time.sleep(0.5)
                        # detect again
                        break
                else:
                    # Small block
                    if not color_line_small[block.color]:
                        target_pos = color_place_small[block.color]
                        self.auto_place(target_pos= target_pos, target_orientation = 0,isbig = False, save_time=False, vertical = True)
                        color_line_small[block.color] = True
                        self.sky_walker()
                        time.sleep(0.5)
                    else:
                        print("----------------------We fuck up, something  wrong---------------------")
                        self.rxarm.gripper.release()
                        time.sleep(0.5)
                        self.sky_walker()
                        time.sleep(0.5)
                        # detect again
                        break
        self.initialize_rxarm()
        time.sleep(2)
        self.rxarm.arm.go_to_sleep_pose(moving_time = 1.5,
                            accel_time=0.5,
                            blocking=True)
        print("##################### Line 'em up Complete #####################")

    # Event 4:Stack 'em high!
    def stack_em_high(self):
        self.current_state = "stack_em_high"
        self.next_state = "idle"
        print("##################### Stack 'em high Start #####################")
        print("---------------------- Clear Stage1 Start ---------------------")
        x_big, x_small = 170,-150
        y_big, y_small = 0,0
        block_offset = 65
        big_counter1,small_counter1 = 1,1

        while True:
            self.sky_walker(camera_clean=True)
            time.sleep(1)
            self.detection.run(boundary=self.camera.boundary[0:2], only_blocks=True)
            time.sleep(0.1)
            isStack = False
            for block in self.detection.blocks:
                if block.stack:
                    isStack = True
                    block.depth = block.depth
                    # if it's stack 2 high, place the top one first
                    print("=========== Remove a Stack block Start ",block.world_xyz," =============")

                    if block.type == 'big':
                        self.auto_pick(target_pos=block.world_xyz,block_ori = block.orientation ,isbig=True,isStack=True,depth=block.depth)
                        if self.rxarm.get_gripper_position() <= 0.02:
                            print("Actually it is a Small block")
                            self.sky_walker()
                            time.sleep(0.2)
                            self.auto_place(target_pos=[x_small,y_small,0],target_orientation = 0,isbig=False,vertical=False)
                            self.sky_walker()
                            small_counter1 += 1
                            x_small -=60
                        else:
                            self.sky_walker()
                            time.sleep(0.2)
                            self.auto_place(target_pos=[x_big,y_big,0],target_orientation = 0,isbig=True,vertical=False)
                            self.sky_walker()
                            big_counter1 += 1
                            x_big +=65
                    else:
                        self.auto_pick(target_pos=block.world_xyz,block_ori = block.orientation ,isbig=False,isStack=True,depth=block.depth)
                        time.sleep(0.2)
                        if self.rxarm.get_gripper_position() >= 0.02:
                            print("Actually it is a Big block")
                            self.sky_walker()
                            time.sleep(0.2)
                            self.auto_place(target_pos=[x_big,y_big,0],target_orientation = 0,isbig=True,vertical=False)
                            self.sky_walker()
                            big_counter1 += 1
                            x_big +=60
                        else:
                            self.sky_walker()
                            time.sleep(0.2)
                            self.auto_place(target_pos=[x_small,y_small,0],target_orientation = 0,isbig=False,vertical=False)
                            self.sky_walker()
                            small_counter1 += 1
                            x_small -=60
                        
                    print("=========== Remove a Stack block Comptele =============")                   
            
            if not isStack:
                break              
        print("---------------------- Clear Stage1 Complete ---------------------")

        print("---------------------- Detect Stage Start ---------------------")
        # Detect blocks in the plane
        while True:
            time.sleep(0.2)
            self.sky_walker(camera_clean=True)
            time.sleep(1.5)
            self.detection.run(boundary=self.camera.boundary[0:2], only_blocks=True)
            time.sleep(0.1)
            
            # Define a custom order for colors
            color_order = {"red": 0, "orange": 1, "yellow": 2, "green": 3, "blue": 4, "purple": 5, None:6}

            # Sort the list of blocks by color
            blocks = self.detection.blocks
            sorted_blocks = sorted(blocks, key=lambda x: color_order.get(x.color, len(color_order)))
            color_counter = {"red": 0, "orange": 0, "yellow": 0, "green": 0, "blue": 0, "purple": 0}

            if len(sorted_blocks) == 12:
                print("Good! There are 12 blocks")
                for block in sorted_blocks:
                    color_counter[block.color] += 1
                for color in color_counter:
                    print("color: ",color_counter[color])
                if all(value == 2 for value in color_counter.values()):
                    print("Good! There are 2 * 6, that's what we want")
                    break

        print("---------------------- Detect Stage Complete ---------------------")
        
        print("---------------------- stack Stage Start ---------------------")
        big_counter2,small_counter2 = 0,0
        z_big, z_small = 0,0
        for block in sorted_blocks:
            if block.type == "small":
                print("================ Small Block No.",small_counter2," ",
                    block.color,"=========================")
                block.world_xyz_copy = list(block.world_xyz)
                block.world_xyz_copy[2] = 20
                vertical_pick = False
                if block.world_xyz_copy[1] > -20 and block.world_xyz_copy[1] < 25:
                    vertical_pick = True
                self.auto_pick(target_pos=block.world_xyz_copy,block_ori = block.orientation ,isbig=False,vertical=vertical_pick)
                self.sky_walker()
                time.sleep(0.2)
                if self.rxarm.get_gripper_position() >= 0.02:
                    print("Actually it is a Big block")
                    self.auto_place(target_pos=[200,-125,z_big],target_orientation = 0,isbig=True)
                    self.sky_walker()
                    big_counter2 +=1
                    z_big += 38
                else:
                    time.sleep(1.2)
                    self.auto_place(target_pos=[-250,-125,z_small],target_orientation = 0,isbig=False)
                    # self.horizontal_auto_place(target_pos=[-300,-125,z_small])
                    self.sky_walker()
                    small_counter2 += 1
                    z_small += 26

            elif block.type == "big":
                print("================ Big Block No.",big_counter2," ",
                    block.color,"=========================")
                block.world_xyz_copy = list(block.world_xyz)
                block.world_xyz_copy[2] = 35
                self.auto_pick(target_pos=block.world_xyz_copy,block_ori = block.orientation ,isbig=True)
                time.sleep(0.3)
                self.sky_walker()
                time.sleep(0.2)
                if self.rxarm.get_gripper_position() <= 0.02:
                    print("Actually it is a Small block")
                    time.sleep(1.2)
                    self.auto_place(target_pos=[-250,-125,z_small],target_orientation = 0,isbig=False)
                    # self.horizontal_auto_place(target_pos=[-300,-125,z_small])
                    self.sky_walker()
                    small_counter2 +=1
                    z_small +=26
                else:
                    self.auto_place(target_pos=[200,-125,z_big],target_orientation = 0,isbig=True)
                    self.sky_walker()
                    big_counter2 += 1
                    z_big += 38
        self.initialize_rxarm()
        time.sleep(1)
        print("---------------------- Stack Stage Complete ---------------------")
        self.initialize_rxarm()
        time.sleep(2)
        self.rxarm.arm.go_to_sleep_pose(moving_time = 1.5,
                            accel_time=0.5,
                            blocking=True)
        print("##################### Stack 'em high Complete #####################")
    
    # Event 5:To the sky!
    def to_the_sky(self):
        self.current_state = "to_the_sky"
        self.next_state = "idle"
        print("##################### To the Sky #####################")
        # Detect blocks in the plane
        self.detection.run(boundary=self.camera.boundary[2:4])
        time.sleep(1.5)
        self.initialize_rxarm()
        time.sleep(2)

        while not self.detection.blocks:
            print("There is no blocks in the workspace!!")
            time.sleep(1)
 
        block_counter = 1
        y_offset = 0
        x_offset = 0
        for block in self.detection.blocks:
            print("============ Start Block NO.",block_counter," =================")
            block.depth = 30
            self.auto_pick(target_pos=block.world_xyz, block_ori = block.orientation )

            # if pick nothing, continue
            if self.rxarm.get_gripper_position() < 0.019:
                self.rxarm.gripper.release()
                time.sleep(1)
                continue

            # to the sky!!!
            if block_counter > 5:
                if block_counter > 8:
                    self.sky_walker(task5=True,first_to_left=True)
                else:
                    self.sky_walker(task5=True)

            # offset for z
            big_z = block_counter * 38 - 30
            if block_counter == 6:
                big_z = big_z - 30
            elif block_counter == 7:
                big_z = big_z - 25
            elif block_counter == 8:
                big_z = big_z - 20
            elif block_counter == 9 or block_counter == 10:
                big_z = big_z - 18
            elif block_counter > 10:
                big_z = big_z - 16

            # offset for x and y
            if block_counter ==1:
                y_offset = 0  
            elif block_counter ==3 or block_counter == 2:
                y_offset = - 5
                x_offset = - 5
            # elif block_counter > 5 and block_counter <= 9:
            #     y_offset = - 1 * block_counter
            #     x_offset = -1* block_counter

            if block_counter > 5:
                issuperhigh = False
                if block_counter > 9:
                    issuperhigh = True
                self.horizontal_auto_place(target_pos=[200,225 + y_offset,big_z],superhigh=issuperhigh,isTask5 = True)
            else:
                self.auto_place(target_pos=[200 + x_offset,225 + y_offset,big_z],target_orientation = np.pi/4,isTask5 = True)

            # to the sky!!!
            if block_counter > 4 and block_counter < 8:
                self.sky_walker(task5=True)
            elif block_counter >=8:
                self.sky_walker(task5=True,post_to_left=True)   
            
            if block_counter == 12:
                break

            print("============ Coomplet Block NO.",block_counter," =================")
            block_counter = block_counter + 1
       
        self.initialize_rxarm()
        time.sleep(2)
        self.rxarm.arm.go_to_sleep_pose(moving_time = 1.5,
                            accel_time=0.5,
                            blocking=True)
        print("##################### To the sky Never Ends #####################")


class StateMachineThread(QThread):
    """!
    @brief      Runs the state machine
    """
    updateStatusMessage = pyqtSignal(str)
    
    def __init__(self, state_machine, parent=None):
        """!
        @brief      Constructs a new instance.

        @param      state_machine  The state machine
        @param      parent         The parent
        """
        QThread.__init__(self, parent=parent)
        self.sm=state_machine

    def run(self):
        """!
        @brief      Update the state machine at a set rate
        """
        while True:
            self.sm.run()
            self.updateStatusMessage.emit(self.sm.status_message)
            time.sleep(0.05)