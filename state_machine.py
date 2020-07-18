"""!
The state machine that implements the logic.
"""

import time
import numpy as np
import cv2
from kinematics import board_z, block_size, board_width
import color as colorlib #the file for color recognition

class StateMachine():
    """!
    @brief      This class describes a state machine.

                TODO: Add states and state functions to this class to implement all of the required logic for the armlab
    """

    def __init__(self, rexarm, planner, kinect):
        """!
        @brief      Constructs a new instance.

        @param      rexarm   The rexarm
        @param      planner  The planner
        @param      kinect   The kinect
        """
        self.rexarm = rexarm
        self.tp = planner
        self.kinect = kinect
        self.status_message = "State: Idle"
        self.current_state = "idle"
        self.next_state = "idle"
        self.newColorPoints = np.array([], dtype=np.uint8) #arrays to hold the new color points to be added
        self.newColorPointsRGB = np.array([], dtype=np.uint8)
        self.saveColor = None
        self.waypoints = [
            [0.0,          0.0,            0.0,            0.0,          0.0,        0.0],
            [np.pi * 0.1,   0.0,            np.pi / 2,      0.0,         0.0,        0.0],
            [np.pi * 0.25,  np.pi / 2,      -np.pi / 2,     np.pi / 2,   0.0,        0.0],
            [np.pi * 0.4,   np.pi / 2,      -np.pi / 2,     0.0,         0.0,        0.0],
            [np.pi * 0.55,  0,              0,              0,           0.0,        0.0],
            [np.pi * 0.7,   0.0,            np.pi / 2,      0.0,         0.0,        0.0],
            [np.pi * 0.85,  np.pi / 2,      -np.pi / 2,     np.pi / 2,   0.0,        0.0],
            [np.pi,         np.pi / 2,      -np.pi / 2,     0.0,         0.0,        0.0],
            [0.0,           np.pi / 2,      np.pi / 2,      0.0,         0.0,        0.0],
            [np.pi / 2,     -np.pi / 2,     np.pi / 2,      0.0,         0.0,        0.0]]

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
        if self.next_state == "initialize_rexarm":
            self.initialize_rexarm()

        if self.next_state == "idle":
            self.idle()

        if self.next_state == "estop":
            self.estop()

        if self.next_state == "execute_tp":
            self.execute_tp()

        if self.next_state == "execute":
            self.execute()

        if self.next_state == "calibrate":
            self.calibrate()

        if self.next_state == "manual":
            self.manual()

        if self.next_state == "reset":
            self.reset()

        if self.next_state == "teach":
            self.teach()

        if self.next_state == "color":
            self.addColorPoints()
        if self.next_state == "save_color":
            self.saveColorPoints()

    """Functions run for each state"""

    def manual(self):
        """!
        @brief      Manually control the Rexarm
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
        self.status_message = "EMERGENCY STOP - Check Rexarm and restart program"
        self.current_state = "estop"
        self.rexarm.disable_torque()

    def execute(self):
        """!
        @brief      Go through all waypoints
        """
        self.status_message = "State: Execute - Executing motion plan"
        self.current_state = "execute"
        self.next_state = "idle"
        f = open("rexarm_pos_data.csv", 'w+')
        f.close()
        self.rexarm.collect_flag = True
        for wp in self.waypoints:
            # Ensure the correct number of joint angles
            full_wp = [0.0] * self.rexarm.num_joints
            full_wp[0:len(wp)] = wp
            # TODO: Set the positions and break if estop is needed
            self.rexarm.set_positions(full_wp)
            if self.next_state == "estop":
                break
            time.sleep(2)

        self.rexarm.collect_flag = False

    def execute_tp(self):
        """!
        @brief      Go through all waypoints with the trajectory planner.
        """
        self.status_message = "State: Execute TP - Executing Motion Plan with trajectory planner"
        self.current_state = "execute_tp"
        self.next_state = "idle"
        waypoints = []
        f = open("rexarm_pos_data.csv", 'w+')
        f.close()
        self.rexarm.collect_flag = True
        for wp in self.waypoints:
            full_wp = [0.0] * self.rexarm.num_joints
            full_wp[0:len(wp)] = wp
            waypoints.append(full_wp)
            # TODO: Send the waypoints to the trajectory planner and break if estop
            self.tp.set_initial_wp()
            self.tp.set_final_wp(wp)
            self.tp.go()
            if self.next_state == "estop":
                self.rexarm.collect_flag = False
                break
            time.sleep(2)

        self.rexarm.collect_flag = False

    def addColorPoints(self):
        self.current_state = "color"
        if self.saveColor == None: #if this is new, clear the array and get color
            self.newColorPoints = np.array([])
            self.newColorPointsRGB = np.array([])
            self.status_message = "Add the color name in the terminal."
            self.saveColor = input("Type the name of the color you want to add: ")
            self.saveColor = self.saveColor.lower()
        self.status_message = ("Click on " + self.saveColor +
                               " blocks. Right click to discard, click \"Add Color Points\" again to save.")
        # don't set next state: stays in color state until button is pressed again
        if self.kinect.new_click:
            self.kinect.new_click = False
            newPoint = self.kinect.VideoFrameHSV[self.kinect.last_click[1], self.kinect.last_click[0], :].reshape((1,3))
            newPointRGB = self.kinect.VideoFrame[self.kinect.last_click[1], self.kinect.last_click[0], :].reshape((1,3))
            if self.newColorPoints.size > 0:
                self.newColorPoints = np.concatenate((self.newColorPoints, newPoint), axis=0)
                self.newColorPointsRGB = np.concatenate((self.newColorPointsRGB, newPointRGB), axis=0)
            else:
                self.newColorPoints = newPoint.copy()
                self.newColorPointsRGB = newPointRGB.copy()
        if self.kinect.new_rclick:
            self.kinect.new_rclick = False
            self.status_message = "Canceled color points"
            self.next_state = "idle"
            self.saveColor = None # reset it so we have to add it again
            time.sleep(1)

    def saveColorPoints(self):
        self.next_state = "idle"
        points = np.load("data/" + self.saveColor + "_HSV.npy")
        pointsRGB = np.load("data/" + self.saveColor + "_RGB.npy")
        if points.size == 0: # if this is the first time it's being saved
            points = self.newColorPoints.copy()
            pointsRGB = self.newColorPointsRGB.copy()
        else:
            points = np.concatenate((points, self.newColorPoints), axis=0)
            pointsRGB = np.concatenate((pointsRGB, self.newColorPointsRGB), axis=0)
        np.save("data/" + self.saveColor + "_HSV.npy", points)
        np.save("data/" + self.saveColor + "_RGB.npy", pointsRGB)

        print("Added to", self.saveColor, f": HSV:\n{self.newColorPoints}\nRGB:\n{self.newColorPointsRGB}")
        for color in colorlib.colors:
            color.load() # reload to include the new data
        self.saveColor = None # reset it so we have to add again

    def calibrate(self):
        """!
        @brief      Gets the calibration clicks
        """
        self.current_state = "calibrate"
        self.next_state = "idle"
        self.kinect.new_click = False
        self.kinect.new_rclick = False

        location_strings = ["lower left corner of board",
                            "upper left corner of board",
                            "upper right corner of board",
                            "lower right corner of board",
                            "center of shoulder motor"]
        i = 0
        for j in range(4):
            self.status_message = "Calibration - click %s in RGB image" % location_strings[j]
            while (i <= j):
                if(self.kinect.new_click == True):
                    self.kinect.rgb_click_points[i] = self.kinect.last_click.astype(np.float32)
                    i = i + 1
                    self.kinect.new_click = False
            time.sleep(0.05)

        # i = 0
        # for j in range(5):
        #     self.status_message = "Calibration - Click %s in depth image" % location_strings[j]
        #     while (i <= j):
        #         if(self.kinect.new_click == True):
        #             self.kinect.depth_click_points[i] = self.kinect.last_click.copy()
        #             i = i + 1
        #             self.kinect.new_click = False

        """Perform camera calibration here"""
        b_w = board_width / 2  # half of board side length

        # model_points = np.array([[-b_w, -b_w, board_z], [-b_w, b_w, board_z], [b_w, b_w, board_z], [b_w, -b_w, board_z], [0,0,0.235 + board_z]])
        model_points = np.array([[-b_w, -b_w, board_z], [-b_w, b_w, board_z], [b_w, b_w, board_z], [b_w, -b_w, board_z]])

        (success, rot_vec, trans_vec) = cv2.solvePnP(model_points, self.kinect.rgb_click_points[0:4,:],
                                                     np.linalg.inv(self.kinect.inv_intrinsic), None, flags=cv2.SOLVEPNP_ITERATIVE)
        if not success:
            print("Extrinsic sucessful?", success)
            return -1

        extrinsic = np.identity(4)

        extrinsic[0:3,0:3] = cv2.Rodrigues(rot_vec)[0]
        extrinsic[0:3,3:4] = trans_vec
        self.kinect.inv_extrinsic = np.linalg.inv(extrinsic)
        np.save("util/extrinsic.npy", self.kinect.inv_extrinsic)
        self.kinect.kinectCalibrated = True
        self.status_message = "Calibration - completed calibration"
        self.kinect.getWorkspaceBoundary()
        time.sleep(1)

    def initialize_rexarm(self):
        """!
        @brief      Initializes the rexarm.
        """
        self.current_state = "initialize_rexarm"

        if not self.rexarm.initialize():
            print('Failed to initialize the rexarm')
            self.status_message = "State: failed to initialize the rexarm!"
            time.sleep(5)
        self.next_state = "idle"

    def reset(self):
        self.waypoints = []
        self.status_message = "Waypoints reset!"
        time.sleep(1)
        if self.next_state == "reset":
            self.next_state = "idle"

    def teach(self):
        self.waypoints.append(self.rexarm.position_fb)
        self.status_message = "Position added!"
        time.sleep(1)
        if self.next_state == "teach":
            self.next_state = "idle"
