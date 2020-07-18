#!/usr/bin/python3
"""!
Test the rexarm

TODO: Use this file and modify as you see fit to test the rexarm. You can specify to use the trajectory planner and
which config file to use on the command line use -h from help.
"""
import os
script_path = os.path.dirname(os.path.realpath(__file__))
os.sys.path.append(os.path.realpath(script_path + '/../'))
import sys
import time
from rexarm import Rexarm
import numpy as np
import argparse
from trajectory_planner import TrajectoryPlanner

# Parse cmd line
parser = argparse.ArgumentParser()
parser.add_argument('-t', '--trajectory_planner', action='store_true')
parser.add_argument('-c', '--config_file', type=str, default=script_path+'/../config/rexarm_config.csv')
args = parser.parse_args()

rexarm = Rexarm()
rexarm.initialize(config_file=args.config_file)
if not rexarm.initialized:
    print('Failed to initialized the rexarm')
    sys.exit(-1)

rexarm.set_speeds_normalized_all(0.5)

tmp_waypoints = [
    [0.0,           0.0,            0.0,            0.0],
    [np.pi * 0.1,   0.0,            np.pi / 2,      0.0],
    [np.pi * 0.25,  np.pi / 2,      -np.pi / 2,     np.pi / 2],
    [np.pi * 0.4,   np.pi / 2,      -np.pi / 2,     0.0],
    [np.pi * 0.55,  0,              0,              0],
    [np.pi * 0.7,   0.0,            np.pi / 2,      0.0],
    [np.pi * 0.85,  np.pi / 2,      -np.pi / 2,     np.pi / 2],
    [np.pi,         np.pi / 2,      -np.pi / 2,     0.0],
    [0.0,           np.pi / 2,      np.pi / 2,      0.0],
    [np.pi / 2,     -np.pi / 2,     np.pi / 2,      0.0]]

waypoints = []
for wp in tmp_waypoints:
    full_wp = [0.0] * rexarm.num_joints
    full_wp[0:len(wp)] = wp
    waypoints.append(full_wp)

if args.trajectory_planner:
    tp = TrajectoryPlanner(rexarm)
    for wp in waypoints:
        tp.set_initial_wp()
        tp.set_final_wp(wp)
        tp.go(max_speed=2.5)
        time.sleep(1.0)
else:
    for wp in waypoints:
        rexarm.set_positions(wp)
        time.sleep(1)

rexarm.disable_torque()
time.sleep(0.1)
