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

                TODO: Add states and state functions to this class to implement all of the required logic for the armlab
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
        self.node = rclpy.create_node('JB_node')
        
        self.replay_pub = self.node.create_publisher(
            Int32,
            'JB_replay',
            10
        )

        self.calibrate_pub = self.node.create_publisher(
            Int32,
            'JB_calibrate',
            10
        )

        self.detect_pub = self.node.create_publisher(
            Int32,
            'JB_Detect',
            10
        )

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
        """!
        @brief      Detect the blocks. If click again, stop detect.

        @rostopic   'JB_detect', Int32, when data is set to 1, start detect.
        """
        self.current_state = "detect"
        self.next_state = "idle"
        msg = Int32()

        self.is_detect = not self.is_detect
        if self.is_detect:
            msg.data = 1
            self.status_message = "Detect color, position, orientation of blocks"
        else:
            msg.data = 0
            self.status_message = "Detect End"

        self.detect_pub.publish(msg)


    def calMoveTime(self, target_joint):
        """!
        @brief    calculate the moving time and accelerate time of motion 
        """
        displacement = np.array(target_joint) - self.rxarm.get_positions()
        angular_v = np.ones(displacement.shape) * (np.pi / 4)
        angular_v[0] = np.pi/3
        angular_t = np.abs(displacement) / angular_v
        move_time = np.max(angular_t)
        if move_time < 0.4:
            move_time = 0.4
        ac_time = move_time / 3
        return move_time, ac_time


    def grab(self):
        """!
        @brief     use IK to grab a block at given position
        """
        self.current_state = "grab"
        self.next_state = "idle"
        if not self.is_detect:
            print("Please click detect first, then click grab again!")
            return None

        while self.camera.blocks == None:
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


    # def get_grab_point(self):
    #     """!
    #     @brief      Judge if it is a valid click point

    #                 return (x,y,z) of block center in the world frame
    #     """
    #     self.camera.new_click = False
    #     while not self.camera.new_click:
    #         time.sleep(0.01)

    #     for block in self.camera.blocks:
    #         if block.inArea(self.camera.last_click):
    #             return self.camera.transformFromImageToWorldFrame((block.center[1], block.center[0])),block.orientation
                
    #     return None, None


    # def grab_or_put_down_a_block(self, click_point, is_grab, ee_orientation = 0):
    #     # click_point[0] is np.array
    #     # orientation should be in radian, which is the ee_orientation

    #     if is_grab:
    #         pose = [click_point[0], click_point[1], click_point[2] - 10, np.pi/3]
    #     else:
    #         pose = [click_point[0], click_point[1], click_point[2] + 30, np.pi/3]


    #     if is_grab:
    #         self.pre_position = pose.copy()
    #         self.pre_position[2] += 100
    #         success1, joint_pos1 = kinematics.IK_geometric(self.pre_position,block_ori=ee_orientation)
    #         success2, joint_pos2 = kinematics.IK_geometric(pose,block_ori=ee_orientation)
    #         if success1:
    #             self.rxarm.set_positions(joint_pos1)
    #             time.sleep(3)
    #             if success2:
    #                 self.rxarm.set_positions(joint_pos2)
    #                 time.sleep(3)
    #             else:
    #                 print("Point cannot access!!")
    #         else:
    #             print("Pre point cannot access!!")
    #         self.rxarm.gripper.grasp()
    #         time.sleep(0.3)
    #     else:
    #         # current_joint_angles = self.rxarm.get_positions()
    #         pose3 = pose.copy()
    #         pose2 = pose.copy()
    #         pose2[2] += 100
    #         success1, joint_pos1 = kinematics.IK_geometric(self.pre_position)
    #         success2, joint_pos2 = kinematics.IK_geometric(pose2)
    #         success3, joint_pos3 = kinematics.IK_geometric(pose3)
    #         if success1:
    #             self.rxarm.set_positions(joint_pos1)
    #             time.sleep(3)
    #             if success2:
    #                 self.rxarm.set_positions(joint_pos2)
    #                 time.sleep(3)
    #                 if success3:
    #                     self.rxarm.set_positions(joint_pos3)
    #                     time.sleep(3)
    #                     self.rxarm.gripper.release()
    #                     time.sleep(0.5)
    #                     self.rxarm.set_positions(joint_pos2)
    #                     time.sleep(3)
    #                 else:
    #                     print("Cannot reach pose3 !")
    #                     self.initialize_rxarm()
    #                     time.sleep(1)
    #             else:
    #                 print("Cannot reach pose2 !")
    #                 self.initialize_rxarm()
    #                 time.sleep(1)
    #         else:
    #             print("Cannot reach pose1 !")
    #             self.initialize_rxarm()
    #             time.sleep(1)

    
    def compute_ee_world_pos(self):
        """!
        @brief      compute end effector world position using DH forward kinematics
        """
        current_joint_angles = self.rxarm.get_positions()
        T_DH = kinematics.FK_dh(dh_params=self.dh_params, joint_angles=current_joint_angles, link=5)
        world_pos = T_DH[0:3,3]
        print(world_pos)
        world_pos.tolist()
        return world_pos



    def pose_compute(self,pos,block_ori):
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

        can_Pick,joint_angles = kinematics.IK_geometric([x,y,z,phi],block_ori=orientaion)
        if can_Pick:
            print("Pose Compute: Success")
            return True,joint_angles
        else:
            for i in range(2):
                x = x * 0.99
                y = y * 0.99
                phi = phi - np.pi*(1.0/30.0)
                can_Pick,joint_angles = kinematics.IK_geometric([x,y,z,phi],block_ori=orientaion)
                print("Pose Compute: adjust")
                if can_Pick:
                    print("Pose Compute: Success")
                    return True, joint_angles
            
            correction_counter = 0
            while not can_Pick:
                phi = phi - np.pi*(1.0/90.0)
                can_Pick,joint_angles = kinematics.IK_geometric([x,y,z,phi],block_ori=orientaion)
                correction_counter = correction_counter + 1
                print("Pose Compute: adjust")
                if can_Pick:
                    print("Pose Compute: Success")
                    return True,joint_angles
                if correction_counter > 5:
                    print("Pose Compute: Failure Can not even reach corrected Pose")
                    print("Pose Compute: The point that cannot reach is: ", [x,y,z])
                    return False,[0,0,0,0,0]
    
    
    def loose_pose_compute(self,pos,block_ori):
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
        
        can_reach,joint_angles = kinematics.IK_geometric([x,y,z,phi],block_ori=orientation)
        if can_reach:
            print("Loose Pose Compute: Success")
            return True,joint_angles
        else:
            for i in range (30):
                x = x * 0.99
                y = y * 0.99
                z = z - 0.1
                phi = phi - np.pi * (1.0/90.0)
                can_reach,joint_angles = kinematics.IK_geometric([x,y,z,phi],block_ori=orientation)
                print("Loose Pose Compute: adjust")
                if can_reach:
                    print("Loose Pose Compute: Success")
                    return True,joint_angles
                
            print("Loose Pose Compute: Failure")
            return False,[0,0,0,0,0]
                

    def horizontal_pose_compute(self,pos,block_ori = 0):
        phi = 0
        can_reach = False
        pos2 = pos[:]
        x = pos2[0]
        y = pos2[1]
        z = pos2[2]

        can_reach,joint_angles = kinematics.IK_geometric([x,y,z,phi])
        if can_reach:
            return True,joint_angles
        else:
            return False, [0,0,0,0,0]


    def auto_pick(self,target_pos,block_ori,isbig = False):
        """!
        @brief      automatically go to a position and pick the block there
        """
        orientation = block_ori
        
        # pos1 is pre_post position, pos2 is pick position, pos3 is post_pick position
        target_pos = kinematics.Target_Pos_Compensation(list(target_pos))
        pos1 = list(target_pos[:])
        pos2 = list(target_pos[:])
        pos3 = list(target_pos[:])
        # print("pos1 shape is: ",len(pos1))
        if isbig:
            pick_offset = 40 # compensation for block height
        else:
            pick_offset = 40
        pick_height = 70 # place gripper above block
        pos2[2] = pos2[2] - pick_offset
        pos1[2] = pos1[2] + pick_height
        pos3[2] = pos3[2] + pick_height

        reachable1, joint_angles1 = self.loose_pose_compute(tuple(pos1),block_ori=orientation)
        reachable2, joint_angles2 = self.pose_compute(pos = tuple(pos2),block_ori=orientation)
        current_joint_angles = self.rxarm.get_positions()
        time.sleep(0.5)

        if reachable1 and reachable2:

            displacement = np.array(joint_angles1) - current_joint_angles

            if np.max(abs(displacement)) > np.pi/2:
                # self.initialize_rxarm()
                # time.sleep(0.5)    
                
                displacement_unit = displacement/3
                pre_pos1 = current_joint_angles + displacement_unit
                pre_pos2 = current_joint_angles + displacement_unit * 2

                move_time,ac_time = self.calMoveTime(pre_pos1)
                self.rxarm.arm.set_joint_positions(pre_pos1,
                                           moving_time = move_time, 
                                           accel_time = ac_time,
                                           blocking = True)
                
                move_time,ac_time = self.calMoveTime(pre_pos2)
                self.rxarm.arm.set_joint_positions(pre_pos2,
                                           moving_time = move_time, 
                                           accel_time = ac_time,
                                           blocking = True)
            elif np.max(abs(displacement)) > np.pi/3:

                displacement_unit = displacement/2
                pre_pos1 = current_joint_angles + displacement_unit
                move_time,ac_time = self.calMoveTime(pre_pos1)
                self.rxarm.arm.set_joint_positions(pre_pos1,
                                           moving_time = move_time, 
                                           accel_time = ac_time,
                                           blocking = True)
                
                


            # go to the pre-picking point
            move_time,ac_time = self.calMoveTime(joint_angles1)
            self.rxarm.arm.set_joint_positions(joint_angles1,
                                           moving_time = move_time, 
                                           accel_time = ac_time,
                                           blocking = True)
            print("Auto Pick: Reach Pos1")
            time.sleep(0.1)
        
            # # go the the picking point not using dichotomy
            # move_time,ac_time = self.calMoveTime(joint_angles2)
            # self.rxarm.arm.set_joint_positions(joint_angles2,
            #                                moving_time = move_time, 
            #                                accel_time = ac_time,
            #                                blocking = True)
            # print("Auto Pick: Reach Pos2")
            # time.sleep(0.1)

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

            move_time,ac_time = self.calMoveTime(joint_angles1)
            self.rxarm.arm.set_joint_positions(joint_angles1,
                                           moving_time = move_time, 
                                           accel_time = ac_time,
                                           blocking = True)
            print("Auto Pick: Reach Pos3")
            print("Auto Pick: Success")
            time.sleep(0.1)
        else:
            print("Auto Pick: Unreachable Position!")
            time.sleep(0.1)



    def auto_place(self,target_pos,target_orientation, isbig = False,save_time = False):
        """!
        @brief      automatically go to a position and place the block there
        """
        orientation = target_orientation
        target_pos = kinematics.Target_Pos_Compensation(list(target_pos))

        # pos1 is pre_post position, pos2 is pick position, pos3 is post_pick position
        pos1 = list(target_pos[:])
        pos2 = list(target_pos[:])
        pos3 = list(target_pos[:])

        # compensation for block height
        if isbig:
            place_offset = 0 
        else:
            place_offset = 5
        place_height = 70 # place gripper above block
        pos2[2] = pos2[2] + place_offset
        pos1[2] = pos1[2] + place_height
        pos3[2] = pos3[2] + place_height

        reachable1, joint_angles1 = self.loose_pose_compute(tuple(pos1),block_ori=orientation)
        reachable2, joint_angles2 = self.pose_compute(pos = tuple(pos2),block_ori=orientation)
        current_joint_angles = self.rxarm.get_positions()
        time.sleep(0.5)

        if reachable1 and reachable2:
           
            displacement = np.array(joint_angles1) - current_joint_angles

            if np.max(abs(displacement)) > np.pi/2:
                # self.initialize_rxarm()
                # time.sleep(0.5)    
                
                displacement_unit = displacement/3
                pre_pos1 = current_joint_angles + displacement_unit
                pre_pos2 = current_joint_angles + displacement_unit * 2

                move_time,ac_time = self.calMoveTime(pre_pos1)
                self.rxarm.arm.set_joint_positions(pre_pos1,
                                           moving_time = move_time, 
                                           accel_time = ac_time,
                                           blocking = True)
                
                move_time,ac_time = self.calMoveTime(pre_pos2)
                self.rxarm.arm.set_joint_positions(pre_pos2,
                                           moving_time = move_time, 
                                           accel_time = ac_time,
                                           blocking = True)
            elif np.max(abs(displacement)) > np.pi/3:

                displacement_unit = displacement/2
                pre_pos1 = current_joint_angles + displacement_unit
                move_time,ac_time = self.calMoveTime(pre_pos1)
                self.rxarm.arm.set_joint_positions(pre_pos1,
                                           moving_time = move_time, 
                                           accel_time = ac_time,
                                           blocking = True)

            move_time,ac_time = self.calMoveTime(joint_angles1)
            self.rxarm.arm.set_joint_positions(joint_angles1,
                                           moving_time = move_time, 
                                           accel_time = ac_time,
                                           blocking = True)
            print("Auto Place: Reach Pos1")
            time.sleep(0.1)
 
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

            self.rxarm.gripper.release()
            time.sleep(0.5)

            move_time,ac_time = self.calMoveTime(joint_angles1)
            self.rxarm.arm.set_joint_positions(joint_angles1,
                                           moving_time = move_time, 
                                           accel_time = ac_time,
                                           blocking = True)
            print("Auto Place: Reach Pos3")
            time.sleep(0.2)
            print("Auto Place: Success")
        else:
            print("Auto Place: Unreachable Position!")
            time.sleep(0.2)

    
    def auto_change_position(self,current_pos):
        """!
        @brief      if the block is stack two high, pick the top one and place it to the ground
        """
        target_pos = current_pos.copy()
        xy_distance = np.sqrt(np.square(current_pos[0])+np.square(current_pos[1]))
        if xy_distance > 250:
            target_pos[0] = target_pos[0]*(7.0/8.0)
            target_pos[1] = target_pos[1]*(7.0/8.0)
            target_pos[2] = 50
        else:
            target_pos[0] = target_pos[0]*(8.0/7.0)
            target_pos[1] = target_pos[1]*(8.0/7.0)
            target_pos[2] = 60
        can_reach,target_joint_angles = self.loose_pose_compute(target_pos,block_ori=0)

        if can_reach:
            time.sleep(1)
            current_joint_angles = self.rxarm.get_positions()
            displacement = np.array(target_joint_angles) - current_joint_angles

            if np.max(abs(displacement)) > np.pi/2:
                # self.initialize_rxarm()
                # time.sleep(0.5)    
                
                displacement_unit = displacement/3
                pre_pos1 = current_joint_angles + displacement_unit
                pre_pos2 = current_joint_angles + displacement_unit * 2

                move_time,ac_time = self.calMoveTime(pre_pos1)
                self.rxarm.arm.set_joint_positions(pre_pos1,
                                           moving_time = move_time, 
                                           accel_time = ac_time,
                                           blocking = True)
                
                move_time,ac_time = self.calMoveTime(pre_pos2)
                self.rxarm.arm.set_joint_positions(pre_pos2,
                                           moving_time = move_time, 
                                           accel_time = ac_time,
                                           blocking = True)
            elif np.max(abs(displacement)) > np.pi/3:

                displacement_unit = displacement/2
                pre_pos1 = current_joint_angles + displacement_unit
                move_time,ac_time = self.calMoveTime(pre_pos1)
                self.rxarm.arm.set_joint_positions(pre_pos1,
                                            moving_time = move_time, 
                                            accel_time = ac_time,
                                            blocking = True)

            move_time,ac_time = self.calMoveTime(target_joint_angles)
            self.rxarm.arm.set_joint_positions(target_joint_angles,
                                           moving_time = move_time, 
                                           accel_time = ac_time,
                                           blocking = True)
            time.sleep(0.2)
            final_pos = self.compute_ee_world_pos()
            print("Auto Change Position: reach bomb position: ", final_pos)
            
            self.rxarm.gripper.release()
            time.sleep(0.5)
            print("Auto Change Position: Success")
        else:
            print("Auto Change Position: Failure")
  



    def kinematics_motion_test(self):
        self.current_state = "motion_test"
        self.next_state = "idle"

        point1 = kinematics.Target_Pos_Compensation([150,150,0])
        point1 = [point1[0],point1[1],point1[2],np.pi/2]
        point2 = kinematics.Target_Pos_Compensation([200,200,0])
        point2 = [point2[0],point2[1],point2[2],np.pi/2]
        point3 = kinematics.Target_Pos_Compensation([275,275,0])
        point3 = [point3[0],point3[1],point3[2],np.pi/2]
        point4 = kinematics.Target_Pos_Compensation([277,277,0])
        point4 = [point4[0],point4[1],point4[2],np.pi/2]

        can_reach1,joint_angles1 = kinematics.IK_geometric(point1)
        can_reach2,joint_angles2 = kinematics.IK_geometric(point2)
        can_reach3,joint_angles3 = kinematics.IK_geometric(point3)
        can_reach4,joint_angles4 = kinematics.IK_geometric(point4)

        if can_reach1:
            print("move to point1")
            # joint_angles1 = kinematics.Joint_Pos_Compensation(joint_angles1)
            # move_time,ac_time = self.calMoveTime(joint_angles1)
            # self.rxarm.arm.set_joint_positions(joint_angles1,
            #                                    moving_time = move_time,
            #                                    accel_time = ac_time,
            #                                    blocking = True)
            # curr_pos = self.rxarm.get_positions()
            # print("current pos is: ",curr_pos)
            self.rxarm.set_positions(joint_angles1)
            time.sleep(2)
            time.sleep(3)
            joint_angles1 = self.rxarm.get_positions()
            joint_angles1[1] = -joint_angles1[1]
            joint_angles1[2] = -joint_angles1[2]
            joint_angles1[3] = -joint_angles1[3]
            # T_DH = kinematics.FK_dh(dh_params=self.dh_params, joint_angles=joint_angles1, link=5)
            # print("T_DH is: ",T_DH)
            pos = self.compute_ee_world_pos()

        self.rxarm.arm.go_to_sleep_pose(moving_time = 1.5,
                              accel_time=0.5,
                              blocking=True)

        if can_reach2:
            print("move to point2")
            # joint_angles2 = kinematics.Joint_Pos_Compensation(joint_angles2)
            # move_time,ac_time = self.calMoveTime(joint_angles2)
            # self.rxarm.arm.set_joint_positions(joint_angles2,
            #                                 moving_time = move_time,
            #                                 accel_time = ac_time,
            #                                 blocking = True)
            self.rxarm.set_positions(joint_angles2)
            time.sleep(2)
            curr_pos = self.rxarm.get_positions()
            print("current pos is: ",curr_pos)
            time.sleep(3)
            joint_angles2 = self.rxarm.get_positions()
            joint_angles2[1] = -joint_angles2[1]
            joint_angles2[2] = -joint_angles2[2]
            joint_angles2[3] = -joint_angles2[3]
            T_DH = kinematics.FK_dh(dh_params=self.dh_params, joint_angles=joint_angles2, link=5)
            print("T_DH is: ",T_DH)
            

        if can_reach3:
            print("move to point3")
            # joint_angles3 = kinematics.Joint_Pos_Compensation(joint_angles3)
            # move_time,ac_time = self.calMoveTime(joint_angles3)
            # self.rxarm.arm.set_joint_positions(joint_angles3,
            #                                 moving_time = move_time,
            #                                 accel_time = ac_time,
            #                                 blocking = True)
            self.rxarm.set_positions(joint_angles3)
            time.sleep(2)
            curr_pos = self.rxarm.get_positions()
            print("current pos is: ",curr_pos)
            time.sleep(5)
            joint_angles3 = self.rxarm.get_positions()
            joint_angles3[1] = -joint_angles3[1]
            joint_angles3[2] = -joint_angles3[2]
            joint_angles3[3] = -joint_angles3[3]
            T_DH = kinematics.FK_dh(dh_params=self.dh_params, joint_angles=joint_angles3, link=5)
            print("T_DH is: ",T_DH)

        if can_reach4:
            print("move to point4")
            # move_time,ac_time = self.calMoveTime(joint_angles4)
            # self.rxarm.arm.set_joint_positions(joint_angles4,
            #                                 moving_time = move_time,
            #                                 accel_time = ac_time,
            #                                 blocking = True)
            self.rxarm.set_positions(joint_angles4)
            time.sleep(2)
            curr_pos = self.rxarm.get_positions()
            print("current pos is: ",curr_pos)
            time.sleep(3)
            joint_angles4 = self.rxarm.get_positions()
            T_DH = kinematics.FK_dh(dh_params=self.dh_params, joint_angles=joint_angles4, link=5)
            print("T_DH is: ",T_DH)
        
        print("Motion Test Done!")



    def safe_pos(self):
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



    # Event 1:Pick'n sort!
    def pick_n_sort(self,istask3 = False):
        if not istask3:
            self.current_state = "pick_n_sort"
            self.next_state = "idle"
            print("##################### Pick 'n sort Start #####################")
               
        block_number_counter = 1
        small_counter,big_counter = 0,0
        small_x , big_x = -160,160
        small_y,big_y = -100,-100

        if istask3:
            block_offset = 80
        else:
            block_offset = 70

        # 1st time deal with 2 floor blocks, 2nd time deal with general blocks
        while True:
            # Detect blocks in the plane
            self.camera.blocks = detectBlocksInRGBImage(self.camera.VideoFrame.copy(), self.camera.DepthFrameRaw,boundary=self.camera.boundary)
            time.sleep(1)

            while self.camera.blocks == None:
                print("There is no blocks in the workspace!!")
                time.sleep(1)
            
            all_block_in_neg_plane = True
            for block in self.camera.blocks:
                block_center, block_orientation = self.camera.transformFromImageToWorldFrame((block.center[1], block.center[0])),block.orientation 
                if block_center[1] > 0:
                    all_block_in_neg_plane = False
            if all_block_in_neg_plane:
                break

            # Initialize place positions - x coordinates           
            self.initialize_rxarm()
            time.sleep(2)

            for block in self.camera.blocks:
                block_center, block_orientation = self.camera.transformFromImageToWorldFrame((block.center[1], block.center[0])),block.orientation 
                
                # print(block_center,block.side)
                if block_center[2] < 150: 
                    print("--------------------- start a block:No.",block_number_counter,"---------------------------")
                    print("block_depth", block_center[2])

                    # Move small blocks in right plane to left planet
                    if block_center[2] > 65:
                        # if it's stack 2 high, place the top one first
                        print("=========== Stack Two ",block_center," =============")
                        self.auto_pick(target_pos=block_center,block_ori = block_orientation,isbig=False)
                        time.sleep(0.2)
                        current_position = self.compute_ee_world_pos()
                        self.auto_change_position(current_pos=current_position)
                        print("=========== Stack Two Comptele =============")
                    
                    else:
                        # if it's stack 1 high, pick and place normally
                        if block.side <= 25:
                            if block_center[0] >= 0 or block_center[1] >= 0:
                                print("=========== Small ",block_center," =============")

                                self.auto_pick(target_pos=block_center,block_ori = block_orientation,isbig=False)
                                time.sleep(0.2)

                                 # compute the place position             
                                if small_counter % 2 == 0:
                                        small_y = -100
                                        if small_counter != 0:
                                            small_x -= block_offset
                                else:
                                    small_y = -100 + block_offset

                                self.auto_place(target_pos=[small_x,small_y,0],target_orientation = 0,isbig=False,save_time=True)
                                small_counter += 1
                                block_number_counter += 1
                        # Move big blocks in left plane to right plane
                        elif block.side >= 35:
                            if block_center[0] <= 0 or block_center[1] >= 0:
                                print("=========== Big ",block_center," =============")
                                
                                self.auto_pick(target_pos=block_center,block_ori = block_orientation,isbig=True)
                                time.sleep(0.2)

                                # compute the place position
                                if big_counter % 2 == 0:
                                    big_y = -100
                                    if big_counter !=0:
                                        big_x += block_offset
                                else:
                                    big_y = -100 + block_offset

                                self.auto_place(target_pos=[big_x,big_y,0],target_orientation = 0,isbig = True,save_time=True)
                                big_counter +=1
                                block_number_counter += 1
                    time.sleep(0.2)
                    # self.initialize_rxarm()
                    # time.sleep(0.5)
                    print("--------------------- end a block ---------------------------")
                    # self.safe_pos()
                else:
                    print("This is April Tag!")
            self.initialize_rxarm()
            time.sleep(0.5)
            self.rxarm.arm.go_to_sleep_pose(moving_time = 1.5,
                              accel_time=0.5,
                              blocking=True)
        print("##################### Pick 'n sort finished #####################")    


    # Event 2:Pick'n stack!
    def pick_n_stack(self):
        self.current_state = "pick_n_stack"
        self.next_state = "idle"
        print("##################### Pick 'n stack Start #####################")
               
        block_number_counter = 1
        small_counter,big_counter = 0,0
        small_x , big_x = -150,150
        small_y,big_y = -100,-100
        small_z,big_z = 0,0

        # 1st time deal with 2 floor blocks, 2nd time deal with general blocks
        while True:
            # Detect blocks in the plane
            self.camera.blocks = detectBlocksInRGBImage(self.camera.VideoFrame.copy(), self.camera.DepthFrameRaw,boundary=self.camera.boundary)
            time.sleep(1)

            while self.camera.blocks == None:
                print("There is no blocks in the workspace!!")
                time.sleep(1)
            
            all_block_in_neg_plane = True
            for block in self.camera.blocks:
                block_center, block_orientation = self.camera.transformFromImageToWorldFrame((block.center[1], block.center[0])),block.orientation 
                if block_center[1] > 0:
                    all_block_in_neg_plane = False
            if all_block_in_neg_plane:
                break

            # Initialize place positions - x coordinates           
            self.initialize_rxarm()
            time.sleep(2)

            for block in self.camera.blocks:
                block_center, block_orientation = self.camera.transformFromImageToWorldFrame((block.center[1], block.center[0])),block.orientation 
                
                # print(block_center,block.side)
                if block_center[2] < 150: 
                    print("--------------------- start a block:No.",block_number_counter,"---------------------------")
                    print("block_depth", block_center[2])

                    # Move small blocks in right plane to left planet
                    if block_center[2] > 65:
                        # if it's stack 2 high, place the top one first
                        print("=========== Stack Two ",block_center," =============")
                        self.auto_pick(target_pos=block_center,block_ori = block_orientation,isbig=False)
                        time.sleep(0.2)
                        current_position = self.compute_ee_world_pos()
                        self.auto_change_position(current_pos=current_position)
                        print("=========== Stack Two Comptele =============")
                    
                    else:
                        # if it's stack 1 high, pick and place normally
                        if block.side <= 25:
                            if block_center[0] >= 0 or block_center[1] >= 0:
                                print("=========== Small ",block_center," =============")

                                self.auto_pick(target_pos=block_center,block_ori = block_orientation,isbig=False)
                                time.sleep(0.2)

                                self.auto_place(target_pos=[small_x,small_y,small_z],target_orientation = 0,isbig=False,save_time=True)
                                small_counter += 1
                                block_number_counter += 1
                                small_z += 20
                        # Move big blocks in left plane to right plane
                        elif block.side >= 35:
                            if block_center[0] <= 0 or block_center[1] >= 0:
                                print("=========== Big ",block_center," =============")
                                
                                self.auto_pick(target_pos=block_center,block_ori = block_orientation,isbig=True)
                                time.sleep(0.2)

                                self.auto_place(target_pos=[big_x,big_y,big_z],target_orientation = 0,isbig = True,save_time=True)
                                big_counter +=1
                                block_number_counter += 1
                                big_z += 38
                    time.sleep(0.2)
                    # self.initialize_rxarm()
                    # time.sleep(0.5)
                    print("--------------------- end a block ---------------------------")
                    # self.safe_pos()
                else:
                    print("This is April Tag!")
            self.initialize_rxarm()
            time.sleep(0.5)
            self.rxarm.arm.go_to_sleep_pose(moving_time = 1.5,
                              accel_time=0.5,
                              blocking=True)
        print("##################### Pick 'n sort finished #####################")  


    # Event 3:Line 'em up!
    def line_em_up(self):
        """!
        @brief      task3
        """
        self.current_state = "line_em_up"
        self.next_state = "idle"
        print("##################### Line 'em up Start #####################")

        self.pick_n_sort(istask3=True)
        print("----------------------Clear Stage Complete")

        # Detect blocks in the plane
        self.camera.blocks = detectBlocksInRGBImage(self.camera.VideoFrame.copy(), self.camera.DepthFrameRaw,boundary=self.camera.boundary)

        while self.camera.blocks == None:
            print("There is no blocks in the workspace!!")
            time.sleep(1)
        
        # Define a custom order for colors
        color_order = {"red": 0, "orange": 1, "yellow": 2, "green": 3, "blue": 4, "purple": 5, None:6}

        # Sort the list of blocks by color
        blocks = self.camera.blocks
        sorted_blocks = sorted(blocks, key=lambda x: color_order.get(x.color, len(color_order)))

        small_x, big_x = -150, -150
        small_y, big_y = 200, 275
        block_counter,small_block_counter,big_block_counter = 1,1,1
        for block in sorted_blocks:
            block_center, block_orientation = self.camera.transformFromImageToWorldFrame((block.center[1], block.center[0])),block.orientation 

            # Filter out possible mis-ditection
            if block_center[2] < 60: 
                # print(block_center)
                # Assume all non-sorted blocks are in the positive plane:    
                print("--------------------- start a block:No.",block_counter,"---------------------------")
                # Line small blocks in color order in the left negative plane
                if block.side <= 25:
                    print("================ Small Block No.",small_block_counter," ",
                          block.color,"=========================")
                    self.auto_pick(target_pos=block_center,block_ori = block_orientation)
                    self.auto_place(target_pos=[small_x,small_y,5],target_orientation = 0)
                    small_x += 50
                    small_block_counter += 1
                # Line big blocks in color order in the right negative plane
                elif block.side >= 35:
                    print("================ Big Block No.",big_block_counter," ",
                          block.color,"=========================")
                    self.auto_pick(target_pos=block_center,block_ori = block_orientation)
                    self.auto_place(target_pos=[big_x,big_y,5],target_orientation = 0)
                    big_x += 50
                    big_block_counter += 1
            print("--------------------- start a block:No.",block_counter,"---------------------------")
            time.sleep(0.2)

        self.initialize_rxarm()
        time.sleep(0.5)
        self.rxarm.arm.go_to_sleep_pose(moving_time = 1.5,
                            accel_time=0.5,
                            blocking=True)
        print("##################### Line 'em up Complete #####################")
        pass

    # Event 4:Stack 'em high!
    def stack_em_high(self):
        self.current_state = "stack_em_high"
        self.next_state = "idle"
        print("##################### Stack 'em high Start #####################")
        # Detect blocks in the plane
        self.camera.blocks = detectBlocksInRGBImage(self.camera.VideoFrame.copy(), self.camera.DepthFrameRaw,boundary=self.camera.boundary)

        while self.camera.blocks == None:
            print("There is no blocks in the workspace!!")
            time.sleep(1)
        
        # Define a custom order for colors
        color_order = {"red": 0, "orange": 1, "yellow": 2, "green": 3, "blue": 4, "purple": 5, None:6}

        # Sort the list of blocks by color
        blocks = self.camera.blocks
        sorted_blocks = sorted(blocks, key=lambda x: color_order.get(x.color, len(color_order)))

        small_z, big_z = 0, 0
        block_counter,small_block_counter,big_block_counter = 1,1,1
        for block in sorted_blocks:
            block_center, block_orientation = self.camera.transformFromImageToWorldFrame((block.center[1], block.center[0])),block.orientation 
            # print("00000000000000000000000000000")
            # print(block_center,block.side)
            # print("00000000000000000000000000000")
            # Filter out possible mis-ditection
            if block_center[2] < 50: 
                # print(block_center)
                # Assume all non-sorted blocks are in the positive plane:
                if block_center [1] >= 0: 
                    # Line small blocks in color order in the left negative plane
                    if block.side <= 25:
                        print("================ Small Block No.",small_block_counter," ",
                          block.color,"=========================")
                        self.auto_pick(target_pos=block_center,block_ori = block_orientation)
                        self.auto_place(target_pos=[-250,-125,small_z],target_orientation = 0)
                        small_z += 38
                    # Line big blocks in color order in the right negative plane
                    elif block.side >= 35:
                        print("================ Big Block No.",big_block_counter," ",
                          block.color,"=========================")
                        self.auto_pick(target_pos=block_center,block_ori = block_orientation)
                        self.auto_place(target_pos=[250,-125,big_z],target_orientation = 0)
                        big_z += 38
            
            time.sleep(1)
        self.initialize_rxarm()
        time.sleep(0.5)
        self.rxarm.arm.go_to_sleep_pose(moving_time = 1.5,
                            accel_time=0.5,
                            blocking=True)
        print("##################### Stack 'em high Complete #####################")
        pass
    
    # Event 5:To the sky!
    def to_the_sky(self):
        self.current_state = "to_the_sky"
        self.next_state = "idle"
        # Detect blocks in the plane
        self.camera.blocks = detectBlocksInRGBImage(self.camera.VideoFrame.copy(), self.camera.DepthFrameRaw,boundary=self.camera.boundary)
  
        while self.camera.blocks == None:
            print("There is no blocks in the workspace!!")
            time.sleep(1)

        big_z = 5
        for block in self.camera.blocks:
            block_center, block_orientation = self.camera.transformFromImageToWorldFrame((block.center[1], block.center[0])),block.orientation 
            self.auto_pick(target_pos=block_center,block_ori = block_orientation)
            self.auto_place(target_pos=[250,175,big_z],target_orientation = 0)
            big_z += 38
            self.initialize_rxarm()
            time.sleep(1)

        self.initialize_rxarm()
        time.sleep(0.5)
        self.rxarm.arm.go_to_sleep_pose(moving_time = 1.5,
                            accel_time=0.5,
                            blocking=True)
        print("##################### To the sky finished Complete #####################")
        pass


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