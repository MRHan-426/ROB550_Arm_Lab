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
        
        if self.next_state == "safe_pos":
            self.safe_pos()

        if self.next_state == "pick_n_sort":
            self.pick_n_sort()

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


    def initialize_rxarm(self):
        """!
        @brief      Initializes the rxarm.
        """
        self.current_state = "initialize_rxarm"
        self.status_message = "RXArm Initialized!"
        self.remembered_waypoint = []
        if not self.rxarm.initialize():
            print('Failed to initialize the rxarm')
            self.status_message = "State: Failed to initialize the rxarm!"
            time.sleep(5)
        self.next_state = "idle"


    def get_grab_point(self):
        """!
        @brief      Judge if it is a valid click point

                    return (x,y,z) of block center in the world frame
        """
        self.camera.new_click = False
        while not self.camera.new_click:
            time.sleep(0.01)

        for block in self.camera.blocks:
            if block.inArea(self.camera.last_click):
                return self.camera.transformFromImageToWorldFrame((block.center[1], block.center[0])),block.orientation
                
        return None, None


    def grab_or_put_down_a_block(self, click_point, is_grab, ee_orientation = 0):
        # click_point[0] is np.array
        # orientation should be in radian, which is the ee_orientation

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


    def pose_compute(pos,block_ori):
        """!
        @brief      compute end effector pose [x,y,z,phi] given x,y,z 
                    and orientation of the block, for general picking situation 
        """
        
        x,y,z = pos.copy()
        orientaion = block_ori.copy()
        phi = np.pi/2
        can_Pick = False

        can_Pick,_ = kinematics.IK_geometric([x,y,z,phi],block_ori=orientaion)
        if can_Pick:
            print("Successfully compute IK pose")
            return True,[x,y,z,phi]
        else:
            for i in range(2):
                x = x * 0.99
                y = y * 0.99
                phi = phi - np.pi*(1/30)
                can_Pick,_ = kinematics.IK_geometric([x,y,z,phi],block_ori=orientaion)
                if can_Pick:
                    print("Successfully compute IK pose")
                    return True, [x,y,z,phi]
            
            correction_counter = 0
            while not can_Pick:
                phi = phi - np.pi*(1/90)
                can_Pick,_ = kinematics.IK_geometric([x,y,z,phi],block_ori=orientaion)
                correction_counter = correction_counter + 1
                if can_Pick:
                    print("Successfully compute IK pose")
                    return True,[x,y,z,phi]
                if correction_counter > 5:
                    print("Failure! Can not even reach corrected Pose")
                    return False,[0,0,0,0]

    
    def loose_pose_compute(pos):
        """!
        @brief      compute pose for those pre_positions or not important positions
                    so there is no strict requirement for phi
        """
        x,y,z = pos
        phi = np.pi/2
        can_reach = False
        
        can_reach,_ = kinematics.IK_geometric([x,y,z,phi])
        if can_reach:
            return True,[x,y,z,phi]
        else:
            correction_counter = 0
            while not can_reach:
                x = x * 0.99
                y = y * 0.99
                z= z - 2
                phi = phi - np.pi * (1/45)
                correction_counter = correction_counter + 1
                can_reach,_ = kinematics.IK_geometric([x,y,z,phi])
                if can_reach:
                    return True,[x,y,z,phi]
                if correction_counter > 20:
                    print("Failure! Can not reach loose Pose")
                    return False,[0,0,0,0]


    def auto_pick(self,target_pos,block_ori):
        """!
        @brief      automatically go to a position and pick the block there
        """
        
        x = target_pos[0].copy()
        y = target_pos[1].copy()
        z = target_pos[2].copy()
        orientation = block_ori.copy()
        
        # pos1 is pre_post position, pos2 is pick position, pos3 is post_pick position
        pos1,pos2,pos3 = target_pos.copy()
        pick_offset = 20 # compensation for block height
        pick_height = 80 # place gripper above block
        pos2[2] = pos2[2] - pick_offset
        pos1[2] = pos1[2] + pick_height
        pos3[2] = pos3[2] + pick_height

        reachable1, joint_angles1 = self.loose_pose_compute(pos1)
        reachable2, joint_angles2 = self.pose_compute(pos2,orientation)

        if reachable1 and reachable2:
            self.rxarm.set_positions(joint_angles1)
            self.rxarm.gripper.release()
            time.sleep(2)
            self.rxarm.set_positions(joint_angles2)
            time.sleep(2)
            self.rxarm.gripper.grasp()
            time.sleep(2)
            self.rxarm.set_positions(joint_angles1)
            time.sleep(1)
            print("Successfully pick the block!")
        else:
            print("Unreachable Position!")

    def safe_pos(self):
        self.status_message = "State: Returning to safe position"
        self.current_state = "safe_pos"
        self.rxarm.set_positions([0,0,-np.pi/2,0,0])
        time.sleep(1)
        self.next_state = "idle"

    # Event 1:Pick'n sort!
    def pick_n_sort(self):
        self.current_state = "pick_n_sort"
        self.next_state = "idle"
        # Detect blocks in the plane
        self.camera.blocks = detectBlocksInDepthImage(self.camera.DepthFrameRaw, intrinsic_matrix=self.camera.intrinsic_matrix, extrinsic_matrix=self.camera.extrinsic_matrix)
        while self.camera.blocks == None:
            print("There is no blocks in the workspace!!")
            time.sleep(1)
        # Differentiate blocks by sizes and catgorize them
        for block in self.camera.blocks:
            block_center, block_orientation = self.camera.transformFromImageToWorldFrame((block.center[1], block.center[0])),block.orientation
            print(block_center)
            if block_center[2] < 40:
                # Move small blocks in right plane to left plane
                if block.side < 20:
                    self.grab_or_put_down_a_block(click_point=block_center, is_grab=True, ee_orientation=block_orientation)
                    self.grab_or_put_down_a_block(click_point=[-150,-75,5], is_grab=False, ee_orientation=block_orientation)
                # Move big blocks in left plane to right plane
                elif block.side > 35:
                    self.grab_or_put_down_a_block(click_point=block_center, is_grab=True, ee_orientation=block_orientation)
                    self.grab_or_put_down_a_block(click_point=[150,-75,5], is_grab=False, ee_orientation=block_orientation)
            self.rxarm.set_positions([0,0,-np.pi/2,0,0])
            time.sleep(1)
        print("Pick 'n sort finished")    

    # Event 2:Pick'n stack!
    def pick_n_stack(self):
        pass

    # Event 3:Line 'em up!
    def line_em_up(self):
        pass

    # Event 4:Stack 'em high!
    def stack_n_high(self):
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