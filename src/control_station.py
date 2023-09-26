#!/usr/bin/python
"""!
Main GUI for Arm lab
"""
import os, sys
script_path = os.path.dirname(os.path.realpath(__file__))

import argparse
import cv2
import numpy as np
import rclpy
import time
from functools import partial

from PyQt5.QtCore import QThread, Qt, pyqtSignal, pyqtSlot, QTimer
from PyQt5.QtGui import QPixmap, QImage, QCursor
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QMainWindow, QFileDialog

from resource.ui import Ui_MainWindow
from rxarm import RXArm, RXArmThread
from camera import Camera, VideoThread
from state_machine import StateMachine, StateMachineThread
""" Radians to/from  Degrees conversions """
D2R = np.pi / 180.0
R2D = 180.0 / np.pi


class Gui(QMainWindow):
    """!
    Main GUI Class

    Contains the main function and interfaces between the GUI and functions.
    """
    def __init__(self, parent=None, dh_config_file=None):
        QWidget.__init__(self, parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        """ Groups of ui commonents """
        self.joint_readouts = [
            self.ui.rdoutBaseJC,
            self.ui.rdoutShoulderJC,
            self.ui.rdoutElbowJC,
            self.ui.rdoutWristAJC,
            self.ui.rdoutWristRJC,
        ]
        self.joint_slider_rdouts = [
            self.ui.rdoutBase,
            self.ui.rdoutShoulder,
            self.ui.rdoutElbow,
            self.ui.rdoutWristA,
            self.ui.rdoutWristR,
        ]
        self.joint_sliders = [
            self.ui.sldrBase,
            self.ui.sldrShoulder,
            self.ui.sldrElbow,
            self.ui.sldrWristA,
            self.ui.sldrWristR,
        ]
        """Objects Using Other Classes"""
        self.camera = Camera()
        print("Creating rx arm...")
        if (dh_config_file is not None):
            self.rxarm = RXArm(dh_config_file=dh_config_file)
        else:
            self.rxarm = RXArm()
        print("Done creating rx arm instance.")
        self.sm = StateMachine(self.rxarm, self.camera)
        """
        Attach Functions to Buttons & Sliders
        TODO: NAME AND CONNECT BUTTONS AS NEEDED
        """
        # Video
        self.ui.videoDisplay.setMouseTracking(True)
        self.ui.videoDisplay.mouseMoveEvent = self.trackMouse
        self.ui.videoDisplay.mousePressEvent = self.calibrateMousePress

        # Buttons
        # Handy lambda function falsethat can be used with Partial to only set the new state if the rxarm is initialized
        #nxt_if_arm_init = lambda next_state: self.sm.set_next_state(next_state if self.rxarm.initialized else None)
        nxt_if_arm_init = lambda next_state: self.sm.set_next_state(next_state)
        self.ui.btn_estop.clicked.connect(self.estop)
        self.ui.btn_init_arm.clicked.connect(self.initRxarm)
        self.ui.btn_torq_off.clicked.connect(
            lambda: self.rxarm.disable_torque())
        self.ui.btn_torq_on.clicked.connect(lambda: self.rxarm.enable_torque())
        self.ui.btn_sleep_arm.clicked.connect(lambda: self.rxarm.sleep())

        #User Buttons
        self.ui.btnUser1.setText("Calibrate")
        self.ui.btnUser1.clicked.connect(partial(nxt_if_arm_init, 'calibrate'))
        # self.ui.btnUser2.setText('Open Gripper')
        # self.ui.btnUser2.clicked.connect(lambda: self.rxarm.gripper.release())
        # self.ui.btnUser3.setText('Close Gripper')
        # self.ui.btnUser3.clicked.connect(lambda: self.rxarm.gripper.grasp())

        self.ui.btnUser2.setText('Open Gripper')
        self.ui.btnUser2.clicked.connect(partial(nxt_if_arm_init, 'open_gripper'))
        self.ui.btnUser3.setText('Close Gripper')
        self.ui.btnUser3.clicked.connect(partial(nxt_if_arm_init, 'close_gripper'))

        self.ui.btnUser4.setText('Execute')
        self.ui.btnUser4.clicked.connect(partial(nxt_if_arm_init, 'execute'))
        self.ui.btnUser5.setText('Record Waypoint')
        self.ui.btnUser5.clicked.connect(partial(nxt_if_arm_init, 'record'))
        self.ui.btnUser6.setText('Replay Waypoint')
        self.ui.btnUser6.clicked.connect(partial(nxt_if_arm_init, 'replay'))
        
        # Sliders
        for sldr in self.joint_sliders:
            sldr.valueChanged.connect(self.sliderChange)
        self.ui.sldrMoveTime.valueChanged.connect(self.sliderChange)
        self.ui.sldrAccelTime.valueChanged.connect(self.sliderChange)
        # Direct Control
        self.ui.chk_directcontrol.stateChanged.connect(self.directControlChk)
        # Status
        self.ui.rdoutStatus.setText("Waiting for input")
        """initalize manual control off"""
        self.ui.SliderFrame.setEnabled(False)
        """Setup Threads"""

        # State machine
        self.StateMachineThread = StateMachineThread(self.sm)
        self.StateMachineThread.updateStatusMessage.connect(
            self.updateStatusMessage)
        self.StateMachineThread.start()
        self.VideoThread = VideoThread(self.camera)
        self.VideoThread.updateFrame.connect(self.setImage)
        self.VideoThread.start()
        self.ArmThread = RXArmThread(self.rxarm)
        self.ArmThread.updateJointReadout.connect(self.updateJointReadout)
        self.ArmThread.updateEndEffectorReadout.connect(
            self.updateEndEffectorReadout)
        self.ArmThread.start()

    """ Slots attach callback functions to signals emitted from threads"""

    @pyqtSlot(str)
    def updateStatusMessage(self, msg):
        self.ui.rdoutStatus.setText(msg)

    @pyqtSlot(list)
    def updateJointReadout(self, joints):
        for rdout, joint in zip(self.joint_readouts, joints):
            rdout.setText(str('%+.2f' % (joint * R2D)))

    ### TODO: output the rest of the orientation according to the convention chosen
    @pyqtSlot(list)
    def updateEndEffectorReadout(self, pos):
        self.ui.rdoutX.setText(str("%+.2f mm" % (pos[0])))
        self.ui.rdoutY.setText(str("%+.2f mm" % (pos[1])))
        self.ui.rdoutZ.setText(str("%+.2f mm" % (pos[2])))
        #self.ui.rdoutPhi.setText(str("%+.2f rad" % (pos[3])))
        #self.ui.rdoutTheta.setText(str("%+.2f" % (pos[4])))
        #self.ui.rdoutPsi.setText(str("%+.2f" % (pos[5])))

    @pyqtSlot(QImage, QImage, QImage, QImage)
    def setImage(self, rgb_image, depth_image, tag_image, grid_image):
        """!
        @brief      Display the images from the camera.

        @param      rgb_image    The rgb image
        @param      depth_image  The depth image
        """
        if (self.ui.radioVideo.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(rgb_image))
        if (self.ui.radioDepth.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(depth_image))
        if (self.ui.radioUsr1.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(tag_image))
        if (self.ui.radioUsr2.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(grid_image))

    """ Other callback functions attached to GUI elements"""

    def estop(self):
        self.rxarm.disable_torque()
        self.sm.set_next_state('estop')

    def sliderChange(self):
        """!
        @brief Slider changed

        Function to change the slider labels when sliders are moved and to command the arm to the given position
        """
        for rdout, sldr in zip(self.joint_slider_rdouts, self.joint_sliders):
            rdout.setText(str(sldr.value()))

        self.ui.rdoutMoveTime.setText(
            str(self.ui.sldrMoveTime.value() / 10.0) + "s")
        self.ui.rdoutAccelTime.setText(
            str(self.ui.sldrAccelTime.value() / 20.0) + "s")
        self.rxarm.set_moving_time(self.ui.sldrMoveTime.value() / 10.0)
        self.rxarm.set_accel_time(self.ui.sldrAccelTime.value() / 20.0)

        # Do nothing if the rxarm is not initialized
        if self.rxarm.initialized:
            joint_positions = np.array(
                [sldr.value() * D2R for sldr in self.joint_sliders])
            # Only send the joints that the rxarm has
            self.rxarm.set_positions(joint_positions[0:self.rxarm.num_joints])

    def directControlChk(self, state):
        """!
        @brief      Changes to direct control mode

                    Will only work if the rxarm is initialized.

        @param      state  State of the checkbox
        """
        if state == Qt.Checked and self.rxarm.initialized:
            # Go to manual and enable sliders
            self.sm.set_next_state("manual")
            self.ui.SliderFrame.setEnabled(True)
        else:
            # Lock sliders and go to idle
            self.sm.set_next_state("idle")
            self.ui.SliderFrame.setEnabled(False)
            self.ui.chk_directcontrol.setChecked(False)

    def trackMouse(self, mouse_event):
        """!
        @brief      Show the mouse position in GUI

                    TODO: after implementing workspace calibration display the world coordinates the mouse points to in the RGB
                    video image.

        @param      mouse_event  QtMouseEvent containing the pose of the mouse at the time of the event not current time
        """

        pt = mouse_event.pos()

        if self.camera.DepthFrameRaw.any() != 0:

            z = self.camera.DepthFrameRaw[pt.y()][pt.x()]
            self.ui.rdoutMousePixels.setText("(%.0f,%.0f,%.0f)" %
                                             (pt.x(), pt.y(), z))

            intrinsic_matrix = self.camera.intrinsic_matrix
            extrinsic_matrix = self.camera.extrinsic_matrix
            
            if self.camera.cameraCalibrated == True:
                T = extrinsic_matrix[:,3]
                R = extrinsic_matrix[:,0:3]

                K2 = extrinsic_matrix
                K1 = intrinsic_matrix

                P_w =  np.linalg.inv(K2) @ np.vstack((z * np.linalg.inv(K1) @ np.array([[pt.x()],[pt.y()],[1]]), 1))
            
            else: # use default extrinsic parameters
                t = np.array([0, 365, 1000])
                R = np.array([[0.999, 0.0094, -0.0426],
                            [0.0, -0.9763,  -0.2164],
                            [-0.0436, 0.2162, -0.9754]])

                u = pt.x()
                v = pt.y()

                fx, fy = intrinsic_matrix[0, 0], intrinsic_matrix[1, 1]
                cx, cy = intrinsic_matrix[0, 2], intrinsic_matrix[1, 2]
                
                x_prime = (u - cx) / fx
                y_prime = (v - cy) / fy

                X_c = z * x_prime
                Y_c = z * y_prime
                Z_c = z
                P_c = np.array([X_c, Y_c, Z_c]).reshape(3, 1)
                t = t.reshape(3,1)

                P_w = R @ P_c + t

            self.ui.rdoutMouseWorld.setText("(%.0f,%.0f,%.0f)" %
                                             (P_w[0], P_w[1], P_w[2]))


    def calibrateMousePress(self, mouse_event):
        """!
        @brief Record mouse click positions for calibration

        @param      mouse_event  QtMouseEvent containing the pose of the mouse at the time of the event not current time
        """
        """ Get mouse posiiton """
        pt = mouse_event.pos()
        self.camera.last_click[0] = pt.x()
        self.camera.last_click[1] = pt.y()
        self.camera.new_click = True
        # print(self.camera.last_click)

    def initRxarm(self):
        """!
        @brief      Initializes the rxarm.
        """
        self.ui.SliderFrame.setEnabled(False)
        self.ui.chk_directcontrol.setChecked(False)
        self.rxarm.enable_torque()
        self.sm.set_next_state('initialize_rxarm')


### TODO: Add ability to parse POX config file as well
def main(args=None):
    """!
    @brief      Starts the GUI
    """
    app = QApplication(sys.argv)
    app_window = Gui(dh_config_file=args['dhconfig'])
    app_window.show()

    # Set thread priorities
    app_window.VideoThread.setPriority(QThread.HighPriority)
    app_window.ArmThread.setPriority(QThread.NormalPriority)
    app_window.StateMachineThread.setPriority(QThread.LowPriority)

    sys.exit(app.exec_())


# Run main if this file is being run directly
### TODO: Add ability to parse POX config file as well
if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument("-c",
                    "--dhconfig",
                    required=False,
                    help="path to DH parameters csv file")
    main(args=vars(ap.parse_args()))