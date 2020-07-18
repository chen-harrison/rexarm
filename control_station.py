#!/usr/bin/python3
"""!
Main GUI for Arm lab
"""

import sys
import cv2
import numpy as np
import time
from functools import partial

from PyQt4.QtCore import (QThread, Qt, pyqtSignal, pyqtSlot, QTimer)
from PyQt4.QtGui import (QPixmap, QImage, QApplication, QWidget, QLabel, QMainWindow, QCursor)

from ui import Ui_MainWindow
from rexarm import Rexarm, RexarmThread
from kinect import Kinect
from trajectory_planner import TrajectoryPlanner
from state_machine import StateMachine


""" Radians to/from  Degrees conversions """
D2R = np.pi / 180.0
R2D = 180.0 / np.pi

"""Threads"""
class VideoThread(QThread):
    """!
    @brief      Kinect video thread.
    """

    # Signals
    updateFrame = pyqtSignal(QImage, QImage, QImage)

    def __init__(self, kinect, parent=None):
        """!
        @brief      Constructs a new instance.

        @param      kinect  The kinect
        @param      parent  The parent
        """
        QThread.__init__(self, parent=parent)
        self.kinect = kinect

    def run(self):
        """!
        @brief      Update rgb and depth images at a set rate
        """
        while True:
            self.kinect.captureVideoFrame()
            self.kinect.captureDepthFrame()
            self.kinect.ColorizeDepthFrame()
            self.kinect.blockDetector()
            rgb_frame = self.kinect.convertQtVideoFrame()
            depth_frame = self.kinect.convertQtDepthFrame()
            depth_filtered_frame = self.kinect.convertQtFilteredFrame()
            # Emit the new frames to be handled by Gui.setImage function
            self.updateFrame.emit(rgb_frame, depth_frame, depth_filtered_frame)
            time.sleep(.03)

class LogicThread(QThread):
    """!
    @brief      Runs the state machine
    """

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
            time.sleep(0.05)

class DisplayThread(QThread):
    """!
    @brief      Update the display
    """

    # Signals
    updateStatusMessage = pyqtSignal(str)
    updateJointReadout = pyqtSignal(list)
    updateEndEffectorReadout = pyqtSignal(list)
    updateJointErrors = pyqtSignal(list)

    def __init__(self, rexarm, state_machine, parent=None):
        """!
        @brief      Constructs a new instance.

        @param      rexarm         The rexarm
        @param      state_machine  The state machine
        @param      parent         The parent
        """
        QThread.__init__(self, parent=parent)
        self.rexarm = rexarm
        self.sm=state_machine

    def run(self):
        """!
        @brief      Update all non image GUI components
        """
        while True:
            # Status message from state machine
            self.updateStatusMessage.emit(self.sm.status_message)
            # Serial errors from rexarm
            self.updateJointErrors.emit(self.rexarm.get_errors())
            # Only get rexarm feedback if initialized
            if self.rexarm.initialized:
                self.updateJointReadout.emit(self.rexarm.position_fb)
                self.updateEndEffectorReadout.emit(self.rexarm.get_wrist_pose())
            time.sleep(0.1)

"""GUI Class"""
class Gui(QMainWindow):
    """!
    Main GUI Class

    Contains the main function and interfaces between the GUI and functions.
    """

    def __init__(self, parent=None):
        QWidget.__init__(self,parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        """ Groups of ui commonents """
        self.error_lcds = [
            self.ui.rdoutBaseErrors,
            self.ui.rdoutShoulderErrors,
            self.ui.rdoutElbowErrors,
            self.ui.rdoutWristErrors,
            self.ui.rdoutWrist2Errors,
            self.ui.rdoutWrist3Errors
        ]
        self.joint_readouts = [
            self.ui.rdoutBaseJC,
            self.ui.rdoutShoulderJC,
            self.ui.rdoutElbowJC,
            self.ui.rdoutWristJC,
            self.ui.rdoutWrist2JC,
            self.ui.rdoutWrist3JC
        ]
        self.joint_slider_rdouts = [
            self.ui.rdoutBase,
            self.ui.rdoutShoulder,
            self.ui.rdoutElbow,
            self.ui.rdoutWrist,
            self.ui.rdoutWrist2,
            self.ui.rdoutWrist3
        ]
        self.joint_sliders = [
            self.ui.sldrBase,
            self.ui.sldrShoulder,
            self.ui.sldrElbow,
            self.ui.sldrWrist,
            self.ui.sldrWrist2,
            self.ui.sldrWrist3
        ]

        """Objects Using Other Classes"""
        self.kinect = Kinect()
        self.rexarm = Rexarm()
        self.tp = TrajectoryPlanner(self.rexarm)
        self.sm = StateMachine(self.rexarm, self.tp, self.kinect)

        """
        Attach Functions to Buttons & Sliders
        TODO: NAME AND CONNECT BUTTONS AS NEEDED
        """
        # Video
        self.ui.videoDisplay.setMouseTracking(True)
        self.ui.videoDisplay.mouseMoveEvent = self.trackMouse
        self.ui.videoDisplay.mousePressEvent = self.calibrateMousePress
        # Buttons
        # Handy lambda function that can be used with Partial to only set the new state if the rexarm is initialized
        nxt_if_arm_init = lambda next_state: self.sm.set_next_state(next_state if self.rexarm.initialized else None)
        self.ui.btn_estop.clicked.connect(self.estop)
        self.ui.btn_init_arm.clicked.connect(self.initRexarm)
        self.ui.btnUser1.setText("Calibrate")
        self.ui.btnUser1.clicked.connect(partial(nxt_if_arm_init, 'calibrate'))
        self.ui.btn_exec.clicked.connect(self.execute)
        self.ui.btnUser2.setText("Reset Waypoints")
        self.ui.btnUser2.clicked.connect(self.reset)
        self.ui.btnUser3.setText("Add Waypoint")
        self.ui.btnUser3.clicked.connect(self.teach)
        self.ui.btnUser4.setText("Gripper Toggle")
        self.ui.btnUser4.clicked.connect(self.toggle_gripper)
        self.ui.btnUser5.setText("Add Color Points")
        self.ui.btnUser5.clicked.connect(self.colPoints)
        self.ui.btnUser6.setText("Click and Grab")
        self.ui.btnUser6.clicked.connect(self.click_grab)
        # Sliders
        for sldr in self.joint_sliders:
            sldr.valueChanged.connect(self.sliderChange)
        self.ui.sldrMaxTorque.valueChanged.connect(self.sliderChange)
        self.ui.sldrSpeed.valueChanged.connect(self.sliderChange)
        # Direct Control
        self.ui.chk_directcontrol.stateChanged.connect(self.directControlChk)
        # Status
        self.ui.rdoutStatus.setText("Waiting for input")
        # Auto exposure
        self.ui.chkAutoExposure.stateChanged.connect(self.autoExposureChk)

        """initalize manual control off"""
        self.ui.SliderFrame.setEnabled(False)

        """Setup Threads"""
        # Rexarm runs its own thread

        # Video
        self.videoThread = VideoThread(self.kinect)
        self.videoThread.updateFrame.connect(self.setImage)
        self.videoThread.start()

        # State machine
        self.logicThread = LogicThread(self.sm)
        self.logicThread.start()

        # Display
        self.displayThread = DisplayThread(self.rexarm, self.sm)
        self.displayThread.updateJointReadout.connect(self.updateJointReadout)
        self.displayThread.updateEndEffectorReadout.connect(self.updateEndEffectorReadout)
        self.displayThread.updateStatusMessage.connect(self.updateStatusMessage)
        self.displayThread.updateJointErrors.connect(self.updateJointErrors)
        self.displayThread.start()

    """ Slots attach callback functions to signals emitted from threads"""

    @pyqtSlot(QImage, QImage, QImage)
    def setImage(self, rgb_image, depth_image, depth_filter_image):
        """!
        @brief      Display the images from the kinect.

        @param      rgb_image    The rgb image
        @param      depth_image  The depth image
        """
        if(self.ui.radioVideo.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(rgb_image))
        if(self.ui.radioDepth.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(depth_image))
        if(self.ui.radioUsr1.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(depth_filter_image))

    @pyqtSlot(list)
    def updateJointReadout(self, joints):
        for rdout, joint in zip(self.joint_readouts, joints):
            rdout.setText(str('%+.2f' % (joint * R2D)))

    @pyqtSlot(list)
    def updateEndEffectorReadout(self, pos):
        self.ui.rdoutX.setText(str("%+.2f" % (pos[0])))
        self.ui.rdoutY.setText(str("%+.2f" % (pos[1])))
        self.ui.rdoutZ.setText(str("%+.2f" % (pos[2])))
        self.ui.rdoutT.setText(str("%+.2f" % (pos[3])))
        # self.ui.rdoutG.setText(str("%+.2f" % (pos[4])))
        # self.ui.rdoutP.setText(str("%+.2f" % (pos[5])))

    @pyqtSlot(list)
    def updateJointErrors(self, errors):
        for lcd, error in zip(self.error_lcds, errors):
            lcd.display(error)

    @pyqtSlot(str)
    def updateStatusMessage(self, msg):
        self.ui.rdoutStatus.setText(msg)


    """ Other callback functions attached to GUI elements"""

    def estop(self):
        self.sm.set_next_state("estop")

    def execute(self):
        # self.sm.set_next_state("execute")
        self.sm.set_next_state("execute_tp")

    def reset(self):
        self.sm.set_next_state("reset")

    def teach(self):
        self.sm.set_next_state("teach")

    def colPoints(self):
        if self.sm.current_state == "color": #if we're already doing colors, now save it
            self.sm.set_next_state("save_color")
        else:
            self.sm.set_next_state("color")

    def toggle_gripper(self):
        self.rexarm.toggle_gripper()

    def click_grab(self):
        # TODO: WRITE FUNCTION HERE!!!
        pass

    def sliderChange(self):
        """!
        @brief Slider changed

        Function to change the slider labels when sliders are moved and to command the arm to the given position
        """
        for rdout, sldr in zip(self.joint_slider_rdouts, self.joint_sliders):
            rdout.setText(str(sldr.value()))

        self.ui.rdoutTorq.setText(str(self.ui.sldrMaxTorque.value()) + "%")
        self.ui.rdoutSpeed.setText(str(self.ui.sldrSpeed.value()) + "%")

        # Do nothing if the rexarm is not initialized
        if self.rexarm.initialized:
            self.rexarm.set_torque_limits([self.ui.sldrMaxTorque.value() / 100.0] * self.rexarm.num_joints)
            self.rexarm.set_speeds_normalized_all(self.ui.sldrSpeed.value() / 100.0)
            joint_positions = np.array([sldr.value() * D2R for sldr in self.joint_sliders])
            # Only send the joints that the rexarm has
            self.rexarm.set_positions(joint_positions[0:self.rexarm.num_joints])

    def directControlChk(self, state):
        """!
        @brief      Changes to direct control mode

                    Will only work if the rexarm is initialized.

        @param      state  State of the checkbox
        """
        if state == Qt.Checked and self.rexarm.initialized:
            # Go to manual and enable sliders
            self.sm.set_next_state("manual")
            self.ui.SliderFrame.setEnabled(True)
        else:
            # Lock sliders and go to idle
            self.sm.set_next_state("idle")
            self.ui.SliderFrame.setEnabled(False)
            self.ui.chk_directcontrol.setChecked(False)

    def autoExposureChk(self, state):
        """!
        @brief      Sets the Kinect auto exposer

        @param      state  State of the checkbox
        """
        if state == Qt.Checked and self.kinect.kinectConnected == True:
            self.kinect.toggleExposure(True)
        else:
            self.kinect.toggleExposure(False)

    def trackMouse(self, mouse_event):
        """!
        @brief      Show the mouse position in GUI

                    TODO: after implementing workspace calibration display the world coordinates the mouse points to in the RGB
                    video image.

        @param      mouse_event  QtMouseEvent containing the pose of the mouse at the time of the event not current time
        """
        if self.kinect.DepthFrameRaw.any() != 0:
            u = mouse_event.pos().x()
            v = mouse_event.pos().y()
            d = self.kinect.DepthFrameRaw[v,u]
            self.ui.rdoutMousePixels.setText("("+str(u)+","+str(v)+","+str(d)+")")
            worldCoords = self.kinect.pix2Glob(np.array([u,v,1])) * 1000 #1000 for mm
            self.ui.rdoutMouseWorld.setText(f"({np.round(worldCoords[0])},{np.round(worldCoords[1])},{np.round(worldCoords[2])})")

    def calibrateMousePress(self, mouse_event):
        """!
        @brief Record mouse click positions for calibration

        @param      mouse_event  QtMouseEvent containing the pose of the mouse at the time of the event not current time
        """

        """ Get mouse posiiton """
        pt = mouse_event.pos()

        if mouse_event.button() == Qt.LeftButton:
            self.kinect.last_click[0] = pt.x()
            self.kinect.last_click[1] = pt.y()
            self.kinect.new_click = True
        elif mouse_event.button() == Qt.RightButton:
            self.kinect.last_rclick[0] = pt.x()
            self.kinect.last_rclick[1] = pt.y()
            self.kinect.new_rclick = True

    def initRexarm(self):
        """!
        @brief      Initializes the rexarm.
        """
        self.sm.set_next_state('initialize_rexarm')
        self.ui.SliderFrame.setEnabled(False)
        self.ui.chk_directcontrol.setChecked(False)


def main():
    """!
    @brief      Starts the GUI
    """
    app = QApplication(sys.argv)
    app_window = Gui()
    app_window.show()
    sys.exit(app.exec_())


# Run main if this file is being run directly
if __name__ == '__main__':
    main()
