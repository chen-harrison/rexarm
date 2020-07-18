#!/usr/bin/python3
"""!
Test the gripper

TODO: Use this file and modify as you see fit to test gripper
"""
import os
script_path = os.path.dirname(os.path.realpath(__file__))
os.sys.path.append(os.path.realpath(script_path + '/../'))
import time
from rexarm import Rexarm

rexarm = Rexarm()
rexarm.initialize(config_file=script_path+'/../config/gripper_only_config.csv')

# Test blocking versions
rexarm.open_gripper_blocking()
rexarm.close_gripper_blocking()
rexarm.toggle_gripper_blocking()

# Test non-blocking versions
rexarm.open_gripper()
time.sleep(1)
rexarm.close_gripper()
time.sleep(1)
rexarm.toggle_gripper()
time.sleep(1)

rexarm.disable_torque()
time.sleep(1)
