"""!
Implements Forward and backwards kinematics with DH parametrs and product of exponentials

TODO: Here is where you will write all of your kinematics functions
There are some functions to start with, you may need to implement a few more
"""

import numpy as np
# expm is a matrix exponential function
from scipy.linalg import expm

board_z = -0.116 # z coord of board wrt origin at shoulder
block_size = 0.0378
board_width = 0.6050

def clamp(angle):
    """!
    @brief      Clamp angles between (-pi, pi]

    @param      angle  The angle

    @return     Clamped angle
    """
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle <= -np.pi:
        angle += 2 * np.pi
    return angle

# ************** DH **************
# Overall transformation matrix
def FK_dh(dh_params, joint_angles, link):
    """!
    @brief      Get the 4x4 transformation matrix from link to world

                TODO: implement this function

                Calculate forward kinematics for rexarm using DH convention

                return a transformation matrix representing the pose of the desired link

                note: phi is the euler angle about the y-axis in the base frame

    @param      dh_params     The dh parameters as a 2D list each row represents a link and has the format [a, alpha, d,
                              theta]
    @param      joint_angles  The joint angles of the links
    @param      link          The link to transform from

    @return     a transformation matrix representing the pose of the desired link
    """
    T = np.eye(4)
    offset = np.pi/2 - 17.18999193*np.pi/180
    # offset = 0
    for i in range(link + 1):
        theta = joint_angles[i]
        if i == 1:
            theta = theta + offset
        if i == 2:
            theta = theta - offset
        d = dh_params[i][1]
        a = dh_params[i][2]
        alpha = dh_params[i][3]
        # print("theta: {}, d: {} , a: {}, alpha: {}".format(theta,d,a,alpha))
        T = T @ get_transform_from_dh(a, alpha, d, theta)
    return T

# Link transformation matrix
def get_transform_from_dh(a, alpha, d, theta):
    """!
    @brief      Gets the transformation matrix from dh parameters.

    TODO: Find the T matrix from a row of a DH table

    @param      a      a meters
    @param      alpha  alpha radians
    @param      d      d meters
    @param      theta  theta radians

    @return     The 4x4 transform matrix.
    """
    T_link = np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
                          [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
                          [0, np.sin(alpha), np.cos(alpha), d],
                          [0, 0, 0, 1]])
    return T_link

# Calculate Euler angles using transformation matrix T
def get_euler_angles_from_T(T):
    """!
    @brief      Gets the euler angles from a transformation matrix.

                TODO: Implement this function return the Euler angles from a T matrix

    @param      T     transformation matrix

    @return     The euler angles from T.
    """
    eps = 1e-10
    if abs(T[0,2] - 0) > eps or abs(T[1,2] - 0) > eps:
        theta = np.arctan2(np.sqrt(1 - T[2,2]**2), T[2,2])
        if np.sin(theta) > 0:
            phi = np.arctan2(T[1,2], T[0,2])
            psi = np.arctan2(T[2,1],-T[2,0])
        else:
            phi = np.arctan2(-T[1,2], -T[0,2])
            psi = np.arctan2(-T[2,1],T[2,0])
    elif abs(T[0,2] - 0) < eps and abs(T[1,2] - 0) < eps:
        theta = 0
        phi = np.arctan2(T[1,0],T[0,0])/2
        psi = phi

    return np.array([phi, theta, psi], dtype=np.float32)

# Calculate joint vector using transformation matrix T
def get_pose_from_T(T):
    """!
    @brief      Gets the pose from T.

                TODO: implement this function return the joint pose from a T matrix of the form (x,y,z,phi) where phi is
                rotation about base frame y-axis

    @param      T     transformation matrix

    @return     The pose from T.
    """
    euler_angles = get_euler_angles_from_T(T)
    theta = euler_angles[1]
    x = T[0, 3]
    y = T[1, 3]
    z = T[2, 3]
    return np.array([x, y, z, theta])

# ************** PoE **************
# Use the joint angles to find transformation matrix g_st
def FK_pox(joint_angles):
    """!
    @brief      Get a 4-tuple (x, y, z, phi) representing the pose of the desired link

                TODO: implement this function, Calculate forward kinematics for rexarm using product of exponential
                formulation return a 4-tuple (x, y, z, phi) representing the pose of the desired link note: phi is the euler
                angle about y in the base frame

    @param      joint_angles  The joint angles

    @return     a 4-tuple (x, y, z, phi) representing the pose of the desired link
    """
    return np.array([0, 0, 0, 0])

# Calculate twist coordinates
def to_s_matrix(w,v):
    """!
    @brief      Convert to s matrix.

    TODO: implement this function
    Find the [s] matrix for the POX method e^([s]*theta)

    @param      w     { parameter_description }
    @param      v     { parameter_description }

    @return     { description_of_the_return_value }
    """
    pass

#************** Inverse kinematics **************
def IK_geometric(dh_params, pose):
    """!
    @brief      Get all possible joint configs that produce the pose.

                TODO: Convert a desired end-effector pose as np.array x,y,z,phi to joint angles

    @param      dh_params  The dh parameters
    @param      pose       The desired pose as np.array x,y,z,phi

    @return     All four possible joint configurations in a numpy array 4x4 where each row is one possible joint
                configuration
    """
    x = pose[0]
    y = pose[1]
    z = pose[2]
    theta = pose[3]
    l1 = dh_params[1][2]
    l2 = dh_params[2][2]
    l3 = dh_params[3][2]
    # l4 = 0.072
    l4 = 0
    beta = np.pi/2 - 17.18999193 * np.pi / 180

    # ********* ELBOW UP *********
    # FORWARD
    h = z - (l3 + l4)*np.sin(theta)
    r = np.sqrt(x**2 + y**2) - (l3 + l4)*np.cos(theta)
    A = np.arccos((h**2 + r**2 + l1**2 - l2**2) / (2 * l1 * np.sqrt(h**2 + r**2)))
    B = np.arccos((l1**2 + l2**2 - (h**2 + r**2)) / (2 * l1 * l2))

    theta1_uf = np.arctan2(pose[1], pose[0])
    theta2_uf = A + np.arctan2(h, r) - beta
    theta3_uf = beta - (np.pi - B)
    theta4_uf = theta - theta2_uf - theta3_uf

    output = np.zeros((4,4))
    output[0,:] = np.array([theta1_uf, theta2_uf, theta3_uf, theta4_uf])

    # BACKWARD
    if theta1_uf > 0:
        theta1_ub = theta1_uf - np.pi
    else:
        theta1_ub = theta1_uf + np.pi

    theta2_ub = np.pi - (A + np.arctan2(h, r)) - beta
    theta3_ub = beta + (np.pi - B)
    theta4_ub = -(theta + theta2_ub + theta3_ub - np.pi)
    output[1,:] = np.array([theta1_ub, theta2_ub, theta3_ub, theta4_ub])

    #********* ELBOW DOWN *********
    # FORWARD
    theta1_df = theta1_uf
    theta2_df = theta2_uf - 2*A
    theta3_df = theta3_ub
    theta4_df = theta - theta2_df - theta3_df
    output[2,:] = np.array([theta1_df, theta2_df, theta3_df, theta4_df])

    # BACKWARD
    theta1_db = theta1_ub
    theta2_db = theta2_ub + 2*A
    theta3_db = theta3_uf
    theta4_db = -(theta + theta2_db + theta3_db - np.pi)
    output[3,:] = np.array([theta1_db, theta2_db, theta3_db, theta4_db])

    return output
