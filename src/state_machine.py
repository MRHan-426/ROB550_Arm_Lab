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
        self.csv_file_path = "/home/student_pm/armlab-f23/src/example.csv"
        self.waypoints = [[-0.7378150601204649 ,  0.7514994246667079 ,  -0.6479373977680756 ,  1.4672342998962642 ,  0.8329812666744316, 1]]
                        # [[-np.pi/2,            -0.5,         -0.3,              0.0,         0.0, 1],
                        #   [0.75*-np.pi/2,        0.5,          0.3,         -np.pi/3,     np.pi/2, 1],
                        #   [0.5*-np.pi/2,        -0.5,         -0.3,        np.pi / 2,         0.0, 1],
                        #   [0.25*-np.pi/2,        0.5,          0.3,         -np.pi/3,     np.pi/2, 1],
                        #   [0.0,                  0.0,          0.0,              0.0,         0.0, 1],
                        #   [0.25*np.pi/2,        -0.5,         -0.3,              0.0,     np.pi/2, 1],
                        #   [0.5*np.pi/2,          0.5,          0.3,         -np.pi/3,         0.0, 1],
                        #   [0.75*np.pi/2,        -0.5,         -0.3,              0.0,     np.pi/2, 1],
                        #   [np.pi/2,              0.5,          0.3,         -np.pi/3,         0.0, 1],
                        #   [0.0,                  0.0,          0.0,              0.0,         0.0, 1]]
        self.node = rclpy.create_node('JB_node')
        
        self.publisher = self.node.create_publisher(
            Int32,
            'JB_replay',
            10
        )

        self.calibrate_pub = self.node.create_publisher(
            Int32,
            'JB_calibrate',
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

                    TODO: Add states and funcitons as needed.
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
        self.publisher.publish(msg)

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
        self.publisher.publish(msg)
        self.next_state = "idle"


    def calibrate(self):
        """!
        @brief      Gets the user input to perform the calibration
        """
        self.current_state = "calibrate"
        self.next_state = "idle"

        """TODO Perform camera calibration routine here"""
        self.status_message = "Calibration - Completed Calibration"

        msg = Int32()
        msg.data = 1
        self.calibrate_pub.publish(msg)
        msg.data = 0
        self.calibrate_pub.publish(msg)


    """ TODO """
    def detect(self):
        """!
        @brief      Detect the blocks
        """
        time.sleep(1)

    def grab(self):
        """!
        @brief     use IK to grab a block at given position
        """

        click_point = self.camera.last_click
        if click_point != np.zeros(2):
            print("task start")

            # start grab (ignore whether there is a block)
            # convert coordinates to world frame 
            pose = [0,0,50,np.pi/2]
            pre_position = pose
            pre_position[2] += 60
            Success1,joint_pos1 = kinematics.IK_geometric(pre_position)
            Success2,joint_pos2 = kinematics.IK_geometric(pose)
            if Success1 and Success2:
                self.current_state = "grab"
                time.sleep(2)
                self.rxarm.set_positions(joint_pos1)
                time.sleep(2)
                self.rxarm.set_positions(joint_pos2)
                time.sleep(2)
                self.next_state = "idle"

        else:
            pass
        




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