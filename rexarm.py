"""!
Implements the Rexarm and Joint class.

The Rexarm class contains:

* last feedback from joints
* functions to command the joints
* functions to get feedback from joints
* functions to do FK and IK
* A run function to update the dynamixel servos
* A function to read the rexarm config file

The Joint class contains:

* Safe serial interfaces to the dynamixel servos
* Clamp function to limit dynamixel joint angles
* Properties of each joint

You will upgrade some functions and also implement others according to the comments given in the code.
"""
import numpy as np
from kinematics import *
import time
import csv
from copy import deepcopy

import os
script_path = os.path.dirname(os.path.realpath(__file__))
os.sys.path.append(os.path.realpath(script_path + '/dynamixel'))
from dynamixel_XL import *
from dynamixel_AX import *
from dynamixel_MX import *
from dynamixel_bus import *

from PyQt4.QtCore import QThread, QMutex

"""
TODO:

Implement the missing functions
add anything you see fit

"""

""" Serial Port Parameters"""
BAUDRATE   = 1000000
DEVICENAME = "/dev/dynamixel".encode('utf-8')

""" Radians to/from  Degrees conversions """
D2R = np.pi / 180.0
R2D = 180.0 / np.pi

class Joint:
    """!
    @brief      This class describes a joint.

                This class handles all serial interfaces except opening the port which is handled by the Rexarm class.
                Additionally it holds all information about the link including angle limits and DH_parameters.
    """
    # Dictionary of strings to servo class
    SERVO = {'MX': DXL_MX, 'AX': DXL_AX, 'XL': DXL_XL}

    def __init__(self, serial_mutex, port_num, name, servo_type, servo_id, min_angle, max_angle, is_gripper, **kwargs):
        """!
        @brief      Construct but do not initialize a new joint

        @param      serial_mutex  QMutex - Mutex to thread lock access to the serial port
        @param      port_num      int - Sserial port number
        @param      name          str - Name of the servo
        @param      servo_type    str - Type of servo to use, acceptable values are {'MX', 'AX', 'XL'}
        @param      servo_id      int - ID of the servo
        @param      min_angle     float - User imposed minimum angle in degrees of the servo
        @param      max_angle     float - User imposed maximum angle in degrees of the servo
        @param      is_gripper    bool - True if this servo is the gripper false otherwise
        @param      kwargs        dict - Dictionary of keyword value pairs that will be turned into readonly properties
                                  of the Joint class, e.g. if kwargs was {'dh_a': 1} then a private variable joint._dh_a
                                  with value 1 will be generated that can be accessed with joint.dh_a
        """
        # Serial
        self._serial_iface = None
        self._serial_mutex = serial_mutex
        self._is_serial_ok = False
        self._serial_errors = 0
        self._port_num = port_num
        self._servo_type = servo_type
        self._servo_id = servo_id
        # Initialized
        self._initialized = False
        # Give the joint a name
        self._name = name
        # Define anlge limits
        self._min_angle = min_angle * D2R
        self._max_angle = max_angle * D2R
        # Gripper
        self._is_gripper = is_gripper
        # Commands
        self._position = None
        self._torque_limit = None
        self._speed = None
        # Feedback
        self._position_fb = 0.0
        self._speed_fb = 0.0
        self._load_fb = 0.0
        self._temp_fb = 0.0
        self._move_fb = 0
        # Get max speed
        self.max_speed = Joint.SERVO[self._servo_type].MAX_SPEED
        # Collect the rest of the kwargs as readonly properties accesable by joint.name
        for name, value in kwargs.items():
            setattr(self, '_' + name, value)
            setattr(Joint, name, property(fget=lambda self, name=name: deepcopy(getattr(self, '_' + name))))

    def initialize(self):
        """!
        @brief      Initializes the servo.

        @return     True if succesful false otherwise
        """
        # Define serial interface and do initial configuration
        self._serial_iface = self._safe_serial(Joint.SERVO[self._servo_type], self._port_num, self._servo_id)
        if self._serial_iface is None:
            self._initialized = False
            return self._initialized

        """ Configure joints """
        self._safe_serial(self._serial_iface.enable_torque)
        if not self._is_serial_ok:
            self._initialized = False
            return self._initialized

        if self._serial_iface.type == 'MX':
            self._safe_serial(self._serial_iface.set_gains, 32, 16, 8)
            if not self._is_serial_ok:
                self._initialized = False
                return self._initialized

        # Set initial commands
        self.position = 0.0
        if not self._is_serial_ok:
            self._initialized = False
            return self._initialized
        self.torque_limit = 0.5
        if not self._is_serial_ok:
            self._initialized = False
            return self._initialized
        self.speed = 0.25
        if not self._is_serial_ok:
            self._initialized = False
            return self._initialized

        # Succesfully initialized
        self._initialized = True
        return self._initialized

    """ Properties """
    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, position):
        pos = self._clamp(position)
        if pos != self._position:
            self._position = pos
            self._safe_serial(self._serial_iface.set_position, self._position)

    @property
    def torque_limit(self):
        return self._torque_limit

    @torque_limit.setter
    def torque_limit(self, torque_limit):
        if self._torque_limit != torque_limit:
            self._torque_limit = torque_limit
            self._safe_serial(self._serial_iface.set_torque_limit, self._torque_limit)

    @property
    def speed(self):
        return self._speed

    @speed.setter
    def speed(self, speed):
        if self._speed != speed:
            self._speed = speed
            self._safe_serial(self._serial_iface.set_speed, self._speed)

    @property
    def position_fb(self):
        position_fb = self._safe_serial(self._serial_iface.get_position)
        if position_fb is not None:
            self._position_fb = position_fb
        return self._position_fb

    @property
    def speed_fb(self):
        speed_fb = self._safe_serial(self._serial_iface.get_speed)
        if speed_fb is not None:
            self._speed_fb = speed_fb
        return self._speed_fb

    @property
    def load_fb(self):
        load_fb = self._safe_serial(self._serial_iface.get_load)
        if load_fb is not None:
            self._load_fb = load_fb
        return self._load_fb

    @property
    def temp_fb(self):
        temp_fb = self._safe_serial(self._serial_iface.get_temp)
        if temp_fb is not None:
            self._temp_fb = temp_fb
        return self._temp_fb

    @property
    def move_fb(self):
        move_fb = self._safe_serial(self._serial_iface.is_moving)
        if move_fb is not None:
            self._move_fb = move_fb
        return self._move_fb

    @property
    def initialized(self):
        return self._initialized

    @property
    def serial_errors(self):
        return self._serial_errors

    @property
    def is_gripper(self):
        return self._is_gripper

    @property
    def min_angle(self):
        return self._min_angle

    @property
    def max_angle(self):
        return self._max_angle

    @property
    def name(self):
        return self._name

    """ Other functions """

    def _clamp(self, pos):
        """!
        @brief      Clamps the angles between joint.min_angle and joint.max_angle

        @param      pos   The unclamped angle

        @return     Clamped angle
        """
        if pos < self.min_angle:
            pos = self.min_angle
        elif pos > self.max_angle:
            pos = self.max_angle
        return pos

    def _safe_serial(self, func, *args, **kwargs):
        """!
        @brief      Safely accesses the serial port.

                    If failure occurs then _is_serial_ok is set to False but if the command is run successfully then
                    _is_serial_ok is set to true. Check the status of _is_serial_ok after using this function to ensure the
                    command ran succesfully.

        @param      func    Serial function
        @param      args    Positional arguments for the serial function
        @param      kwargs  Keyword arguments for the serial function

        @return     The output of the serial function if no errors occured otherwise 0
        """
        self._serial_mutex.lock()
        try:
            out = func(*args, **kwargs)
            self._is_serial_ok = True
            return out
        except Exception as e:
            self._serial_errors += 1
            self._is_serial_ok = False
        finally:
            self._serial_mutex.unlock()

class Rexarm():
    """!
    @brief      This class describes a rexarm.

                The configuration of the rexarm is loaded from a .csv file as described in the Rexarm.initialize.
    """
    # Class constants
    # Convert config file type strings to string to type functions
    STR_TO_TYPE = {
        'bool': lambda s: s.lower() in s.lower() in ['true', '1', 't', 'y', 'yes'],
        'str': str,
        'float': float,
        'int': int
    }

    def __init__(self):
        """!
        @brief      Constructs a new instance.

                    Starts the rexarm run thread but does not initialise the Joints. Call Rexarm.initialize to initialise the
                    Joints.
        """
        # Thread locks
        self._cmd_mutex = QMutex()
        self._serial_mutex = QMutex()
        # Serial bus
        self.dxlbus = None
        # Gripper
        self.gripper = None
        self.gripper_state = True
        # State
        self.estop = False
        self.initialized = False
        # Update rexarm every update_period seconds
        self.update_period = 0.01
        # Cmds
        self.new_speed_cmds = False
        self.speed_cmds = None
        self.new_torque_limit_cmds = False
        self.torque_limit_cmds = None
        self.new_position_cmds = False
        self.position_cmds = None
        # Feedback
        self.position_fb = None
        self.speed_fb = None
        self.load_fb = None
        self.temp_fb = None
        # Joints
        self._joints = []
        self.open_angle = 0
        self.close_angle = -60
        # Data collection flag
        self.collect_flag = False

        # Start run thread
        self.run_thread = RexarmThread(self)
        self.run_thread.start()

    def initialize(self, config_file=script_path+'/config/rexarm_config.csv'):
        """!
        @brief      Initializes the rexarm from given configuration file.

                    Initializes the Joints and serial port

        @param      config_file  The configuration file see Rexarm._config_joints function for more details on the
                                 config file.

        @return     True is succes False otherwise
        """
        self.initialized = False
        # Wait for other threads to finish with the rexarm instead of locking every single call
        time.sleep(1)

        # Get a serial port interface
        self._serial_mutex.lock()
        if self.dxlbus is not None:
            self.dxlbus.close()
        self.dxlbus = DXL_BUS(DEVICENAME, BAUDRATE)
        self.port_num = self.dxlbus.port()
        self._serial_mutex.unlock()

        """ Read in rexarm configs from config file """
        if not self._config_joints(config_file):
            return self.initialized

        """ Commanded Values """
        self.num_joints = len(self._joints)
        self.position = [0.0] * self.num_joints     # degrees
        self.speed = [1.0] * self.num_joints        # 0 to 1
        self.max_torque = [1.0] * self.num_joints   # 0 to 1

        """ Feedback Values """
        self.position_fb = [0.0] * self.num_joints    # degrees
        self.speed_fb = [0.0] * self.num_joints        # 0 to 1
        self.load_fb = [0.0] * self.num_joints         # -1 to 1
        self.temp_fb = [0.0] * self.num_joints         # Celsius
        self.move_fb = [0] *  self.num_joints

        """ Gripper Vaules """
        self.gripper = None
        for joint in self._joints:
            if joint.is_gripper:
                self.gripper = joint

        if self.gripper is not None:
            self.gripper.torque_limit = 1.0
            self.gripper.speed = 0.8

        # Reset estop and initialized
        self.estop = False
        self.initialized = True
        return self.initialized

    def _config_joints(self, config_file):
        """!
        @brief      Configure the Joints and initialize them

                    TODO: Find the physical properties of the Rexarm (such as angle limits and DH parameters) and write a config
                    file for the Rexarm. The config file should be formated as a csv file with the following properties:

                    * The first row lists the exact names of the member variables of the Joint class
                    * The second row list the types of the associated member variables, Rexarm.STR_TO_TYPE dict defines the
                      avaliable types
                    * Each following row defines thoes member variables for a joint in the rexarm
                    * There should be no spaces after a comma

                    You may add any new member variables as you wish to the config file in any order. By default they will be
                    added as a readonly property of the Joint class. If special behaviors are desired for a new member variable
                    then they must be added to the Joint.__init__ parameter list and handled in that function.

        @param      config_file  The configuration file

        @return     True if all joints initialize correctly, False if any joint fails
        """
        self._joints = []
        with open(config_file) as configs:
            reader = csv.reader(configs)
            title_row = next(reader)
            type_strs = next(reader)
            types = [Rexarm.STR_TO_TYPE[s] for s in type_strs]
            for row in reader:
                values = {name: dtype(value) for name, value, dtype in zip(title_row, row, types)}
                joint = Joint(self._serial_mutex, self.port_num, **values)
                self._joints.append(joint)
                if not joint.initialize():
                    return False
        return True

    def _ensure_initialized(func):
        """!
        @brief      Decorator to skip the function if the rexarm is not initialized.

        @param      func  The function to wrap

        @return     The wraped function
        """
        def func_out(self, *args, **kwargs):
            if self.initialized:
                return func(self, *args, **kwargs)
            else:
                print('WARNING: Trying to use the Rexarm before initialized')
        return func_out

    @_ensure_initialized
    def open_gripper(self):
        """!
        @brief      TODO: Tell the gripper to open.
        """
        new_position = self.position_fb
        new_position[5] = self.open_angle
        self.set_positions(new_position)

    @_ensure_initialized
    def open_gripper_blocking(self):
        """!
        @brief      TODO: Open a gripper and block until it is.
        """
        pass

    def is_gripper_open(self):
        """!
        @brief      TODO: Determines if gripper open.

        @return     True if gripper open, False otherwise.
        """
        if abs(self.position_fb[5] - self.open_angle) < 0.2:
            return True
        else:
            return False


    @_ensure_initialized
    def close_gripper(self):
        """!
        @brief      TODO Closes a gripper.
        """
        new_position = self.position_fb
        new_position[5] = self.close_angle
        self.set_positions(new_position)

    @_ensure_initialized
    def close_gripper_blocking(self):
        """!
        @brief      TODO Closes a gripper and block until it is.
        """
        pass

    def is_gripper_close(self):
        """!
        @brief      TODO Determines if gripper close.

        @return     True if gripper close, False otherwise.
        """
        pass

    def toggle_gripper(self):
        if self.is_gripper_open():
            self.open_gripper()
        else:
            self.close_gripper()


    @_ensure_initialized
    def toggle_gripper(self):
        """!
        @brief      TODO Toggle the gripper between open and close
        """
        pass

    @_ensure_initialized
    def toggle_gripper_blocking(self):
        """!
        @brief      TODO Toggle the gripper between open and close and block until done
        """
        pass

    def set_positions(self, joint_angles):
        """!
        @brief      Sets the positions.

        @param      joint_angles  The joint angles
        """
        self._cmd_mutex.lock()
        self.position_cmds = joint_angles
        self.new_position_cmds = True
        self._cmd_mutex.unlock()

    def set_speeds_normalized_all(self, speed):
        """!
        @brief      Sets the speeds normalized all.

        @param      speed  The speed
        """
        self._cmd_mutex.lock()
        self.speed_cmds = [speed] * len(self._joints)
        self.new_speed_cmds = True
        self._cmd_mutex.unlock()

    def set_speeds_normalized(self, speeds):
        """!
        @brief      Sets the speeds normalized.

        @param      speeds  The speeds
        """
        self._cmd_mutex.lock()
        self.speed_cmds = speeds
        self.new_speed_cmds = True
        self._cmd_mutex.unlock()

    def set_speeds(self, speeds):
        """!
        @brief      Sets the speeds.

        @param      speeds  The speeds
        """
        self._cmd_mutex.lock()
        self.speed_cmds = [None] * len(self._joints)
        for i in range(len(self._joints)):
            cmd = abs(speeds[i] / self._joints[i].max_speed)
            if (cmd < 3.0 / 1023.0):
                cmd = 3.0 / 1023.0
            self.speed_cmds[i] = cmd

        self.new_speed_cmds = True
        self._cmd_mutex.unlock()

    def set_torque_limits(self, torques):
        """!
        @brief      Sets the torque limits.

                    Can't be used when estoped.

        @param      torques  The torques
        """
        if self.estop:
            print('WARNING: Torques are zero when estoped')
            return
        self._cmd_mutex.lock()
        self.torque_limit_cmds = torques
        self.new_torque_limit_cmds = True
        self._cmd_mutex.unlock()

    @_ensure_initialized
    def _send_commands(self):
        """!
        @brief      Sends commands.

                    Send the most recent commands to the joints or set torques to zero if estoped.
        """
        # Copy current commands so other threads need not wait for the serial communications
        self._cmd_mutex.lock()
        torque_limit_cmds = None
        speed_cmds = None
        position_cmds = None
        if self.new_torque_limit_cmds:
            torque_limit_cmds = self.torque_limit_cmds.copy()
        if self.new_speed_cmds:
            speed_cmds = self.speed_cmds.copy()
        if self.new_position_cmds:
            position_cmds = self.position_cmds.copy()
        self.new_torque_limit_cmds = False
        self.new_speed_cmds = False
        self.new_position_cmds = False
        self._cmd_mutex.unlock()

        # Set torques to zero when estoped
        if self.estop:
            torque_limit_cmds = [0] * len(self._joints)
            for joint in self._joints:
                joint.torque_limit = 0
            return

        # Write the commands
        for i, joint in enumerate(self._joints):
            if position_cmds is not None:
                joint.position = position_cmds[i]
            # Gripper torque and speed handled seperatly
            if joint.is_gripper:
                continue
            if torque_limit_cmds is not None:
                joint.torque_limit = torque_limit_cmds[i]
            if speed_cmds is not None:
                joint.speed = speed_cmds[i]

    def disable_torque(self):
        """!
        @brief      Disables the torque and estops.
        """
        self.estop = True

    def get_positions(self):
        """!
        @brief      Gets the positions.

        @return     The positions.
        """
        return self.position_fb

    def get_speeds(self):
        """!
        @brief      Gets the speeds.

        @return     The speeds.
        """
        return self.speed_fb

    def get_loads(self):
        """!
        @brief      Gets the loads.

        @return     The loads.
        """
        return self.load_fb

    def get_temps(self):
        """!
        @brief      Gets the temps.

        @return     The temps.
        """
        return self.temp_fb

    def get_moving_status(self):
        """!
        @brief      Gets the moving status.

        @return     The moving status.
        """
        return self.move_fb

    def get_errors(self):
        """!
        @brief      Gets the serial errors.

        @return     The serial errors.
        """
        return [joint.serial_errors for joint in self._joints]

    @_ensure_initialized
    def _get_feedback(self):
        """!
        @brief      Updates all the feedback variables of the joints.
        """
        self.position_fb = [joint.position_fb for joint in self._joints]
        self.speed_fb = [joint.speed_fb for joint in self._joints]
        self.load_fb = [joint.load_fb for joint in self._joints]
        self.temp_fb = [joint.temp_fb for joint in self._joints]
        self.move_fb = [joint.move_fb for joint in self._joints]

    @_ensure_initialized
    def get_wrist_pose(self):
        """!
        @brief      TODO Get the wrist pose.

        @return     The wrist pose as [x, y, z, phi].
        """
        return [0, 0, 0, 0]

    def get_dh_parameters(self):
        """!
        @brief      Gets the dh parameters.

        @return     The dh parameters.
        """
        return [[joint.dh_a, joint.dh_alpha, joint.dh_d, joint.dh_theta] for joint in self._joints]

    @_ensure_initialized
    def run(self):
        """!
        @brief      Update the servos feedback and commands

                    Run in a seperate thread see RexarmThread.
        """
        # Update feedback
        self._get_feedback()
        if self.collect_flag:
            with open("rexarm_pos_data.csv", 'a') as inFile:
                inFile.write(str(time.time()) + ",")
                for pos in self.position_fb:
                    inFile.write(str(pos) + ",")
                inFile.write("\n")
        # Send new commands
        self._send_commands()


class RexarmThread(QThread):
    """!
    @brief      This class describes a rexarm thread.
    """

    def __init__(self, rexarm, parent=None):
        """!
        @brief      Constructs a new instance.

        @param      rexarm  The rexarm
        @param      parent  The parent
        """
        QThread.__init__(self, parent=parent)
        self.rexarm = rexarm

    def run(self):
        """!
        @brief      Updates the Rexarm Joints at a set rate if the rexarm is initialized.
        """
        while True:
            # Get the time this loop should end
            end_time = time.time() + self.rexarm.update_period
            # Update the rexarm
            if self.rexarm.initialized:
                self.rexarm.run()
            # Sleep for the correct amount of time
            if end_time > time.time():
                time.sleep(end_time - time.time())
