"""!
The state machine that implements the logic.
"""
from PyQt5.QtCore import QThread, Qt, pyqtSignal, pyqtSlot, QTimer
import time
import numpy as np
import rclpy

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
        self.waypoints = [
                          [-np.pi/2,            -0.5,         -0.3,              0.0,         0.0, 1],
                          [0.75*-np.pi/2,        0.5,          0.3,         -np.pi/3,     np.pi/2, 1],
                          [0.5*-np.pi/2,        -0.5,         -0.3,        np.pi / 2,         0.0, 1],
                          [0.25*-np.pi/2,        0.5,          0.3,         -np.pi/3,     np.pi/2, 1],
                          [0.0,                  0.0,          0.0,              0.0,         0.0, 1],
                          [0.25*np.pi/2,        -0.5,         -0.3,              0.0,     np.pi/2, 1],
                          [0.5*np.pi/2,          0.5,          0.3,         -np.pi/3,         0.0, 1],
                          [0.75*np.pi/2,        -0.5,         -0.3,              0.0,     np.pi/2, 1],
                          [np.pi/2,              0.5,          0.3,         -np.pi/3,         0.0, 1],
                          [0.0,                  0.0,          0.0,              0.0,         0.0, 1]]


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
        for n in self.waypoints:
            self.rxarm.set_positions(n[:-1])
            time.sleep(3)
        self.next_state = "idle"

    def record(self):
        """!
        """
        self.status_message = "State: record current position"
        if self.remembered_waypoint == []:
            self.rxarm.sleep()
            time.sleep(3)
            self.rxarm.disable_torque()
            self.remembered_waypoint.append(self.waypoints[0])
        else:
            # print("!!!!!!!!!!!!!!!!!!!!")
            # print(self.remembered_waypoint[-1][-1])
            # print("!!!!!!!!!!!!!!!!!!!!")
            curr_pos = self.rxarm.get_positions()
            curr_pos = np.append(curr_pos,self.remembered_waypoint[-1][-1])
            self.remembered_waypoint.append(curr_pos)
        self.next_state = "idle"

    def close_gripper(self) :
        self.status_message = "State: close gripper"
        # if self.remembered_waypoint == []:
        #     self.remembered_waypoint.append(self.waypoints[0])
        self.remembered_waypoint[-1][-1] = 0
        self.rxarm.enable_torque()
        time.sleep(0.2)
        self.rxarm.gripper.grasp()
        time.sleep(0.5)
        self.rxarm.disable_torque()
        # time.sleep(3)
        self.next_state = "idle"

    def open_gripper(self):
        """!
        """
        self.status_message = "State: open gripper"
        if self.remembered_waypoint == []:
            self.remembered_waypoint.append(self.waypoints[0])
        else:
            self.remembered_waypoint[-1][-1] = 1
        self.rxarm.enable_torque()
        time.sleep(0.2)
        self.rxarm.gripper.release()
        time.sleep(1)
        self.rxarm.disable_torque()
        self.next_state = "idle"
        
    def replay(self):

        self.rxarm.enable_torque()
        time.sleep(0.2)
        self.rxarm.gripper.release()
        time.sleep(0.2)

        self.status_message = "State: replay recorded position "
        # print('############')
        # print(self.remembered_waypoint[1:])

        for n in self.remembered_waypoint[1:]:
            self.rxarm.set_positions(n[:-1])
            time.sleep(3)
            if not n[-1]:
                self.rxarm.gripper.grasp()
                time.sleep(0.3)
            else:
                self.rxarm.gripper.release()
                time.sleep(0.3)
        self.next_state = "idle"


    def calibrate(self):
        """!
        @brief      Gets the user input to perform the calibration
        """
        self.current_state = "calibrate"
        self.next_state = "idle"

        """TODO Perform camera calibration routine here"""
        self.status_message = "Calibration - Completed Calibration"

    """ TODO """
    def detect(self):
        """!
        @brief      Detect the blocks
        """
        time.sleep(1)

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