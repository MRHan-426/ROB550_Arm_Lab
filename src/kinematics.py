"""!
Implements Forward and Inverse kinematics with DH parametrs and product of exponentials

TODO: Here is where you will write all of your kinematics functions
There are some functions to start with, you may need to implement a few more
"""

import numpy as np
# expm is a matrix exponential function
from scipy.linalg import expm
import math


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
    @param      link          The link to transform from // link is a number

    @return     a transformation matrix representing the pose of the desired link
    """

    T = np.eye(4)

    for i in range(link):
        a     = dh_params[i][0]
        alpha = dh_params[i][1]
        d     = dh_params[i][2]
        theta = dh_params[i][3] + joint_angles[i]

        A_i = get_transform_from_dh(a, alpha, d, theta)
        T = np.matmul(T, A_i)
    
    return T
    
        

def get_transform_from_dh(a, alpha, d, theta):
    """!
    @brief      Gets the transformation matrix T from dh parameters.

    TODO: Find the T matrix from a row of a DH table

    @param      a      a meters
    @param      alpha  alpha radians
    @param      d      d meters
    @param      theta  theta radians

    @return     The 4x4 transformation matrix.
    """
    A_i = np.array([[cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha) , a * cos(theta)],
                    [sin(theta), cos(theta)*cos(alpha) , -cos(theta)*sin(alpha), a * sin(theta)],
                    [0         , sin(alpha)            , cos(alpha)            , d             ],
                    [0         , 0                     , 0                     , 1             ]])
    return A_i


def get_euler_angles_from_T(T, style = "zyx"):
    """!
    @brief      Gets the euler angles from a transformation matrix.

                TODO: Implement this function return the 3 Euler angles from a 4x4 transformation matrix T
                If you like, add an argument to specify the Euler angles used (xyx, zyz, etc.)

    @param      T     transformation matrix

    @return     The euler angles from T.
    """
    R = T[0:3,0:3]

    def zyx(): # BY DEFAULT
        yaw = np.arctan2(R[1, 0], R[0, 0])
        pitch = np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))
        roll = np.arctan2(R[2, 1], R[2, 2])
        return np.array([yaw, pitch, roll])

    def xyz():
        roll = np.arctan2(-R[1, 2], R[2, 2])
        pitch = np.arctan2(R[0, 2], np.sqrt(R[0, 0]**2 + R[0, 1]**2))
        yaw = np.arctan2(-R[0, 1], R[0, 0])
        return np.array([roll, pitch, yaw])

    def yzx():
        yaw = np.arctan2(-R[0, 1], R[1, 1])
        pitch = np.arctan2(R[2, 1], np.sqrt(R[2, 0]**2 + R[2, 2]**2))
        roll = np.arctan2(-R[2, 0], R[2, 2])
        return np.array([yaw, pitch, roll])

    def xzy():
        roll = np.arctan2(R[1, 0], R[0, 0])
        pitch = np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))
        yaw = np.arctan2(R[0, 2], R[2, 2])
        return np.array([roll, pitch, yaw])

    def zxy():
        yaw = np.arctan2(R[1, 2], R[2, 2])
        pitch = np.arctan2(-R[0, 2], np.sqrt(R[0, 0]**2 + R[0, 1]**2))
        roll = np.arctan2(R[0, 1], R[0, 0])
        return np.array([yaw, pitch, roll])

    def yxz():
        roll = np.arctan2(R[2, 1], R[1, 1])
        pitch = np.arctan2(-R[0, 1], np.sqrt(R[0, 0]**2 + R[0, 2]**2))
        yaw = np.arctan2(R[0, 2], R[0, 0])
        return np.array([roll, pitch, yaw])
    
    switcher = {
        "zyx": zyx,
        "xyz": xyz,
        "yzx": yzx,
        "xzy": xzy,
        "zxy": zxy,
        "yxz": yxz}

    return switcher.get(style, zyx)()


def get_pose_from_T(T, style = "zyx"):
    """!
    @brief      Gets the pose from T.

                TODO: implement this function return the 6DOF pose vector from a 4x4 transformation matrix T

    @param      T     transformation matrix

    @return     The pose vector from T.
    """
    positions = np.array(T[0:3, 3])
    euler_angles = get_euler_angles_from_T(T, style)
    return np.concatenate((positions, euler_angles))



def FK_pox(joint_angles, m_mat, s_lst):
    """!
    @brief      Get a  representing the pose of the desired link

                TODO: implement this function, Calculate forward kinematics for rexarm using product of exponential
                formulation return a 4x4 homogeneous matrix representing the pose of the desired link

    @param      joint_angles  The joint angles
                m_mat         The M matrix
                s_lst         List of screw vectors

    @return     a 4x4 homogeneous matrix representing the pose of the desired link
    """
    pass


def to_s_matrix(w, v):
    """!
    @brief      Convert to s matrix.

    TODO: implement this function
    Find the [s] matrix for the POX method e^([s]*theta)

    @param      w     { parameter_description }
    @param      v     { parameter_description }

    @return     { description_of_the_return_value }
    """
    pass


def IK_geometric(dh_params, pose):
    """!
    @brief      Get all possible joint configs that produce the pose.

                TODO: Convert a desired end-effector pose vector as np.array to joint angles

    @param      dh_params  The dh parameters
    @param      pose       The desired pose vector as np.array 

    @return     All four possible joint configurations in a numpy array 4x4 where each row is one possible joint
                configuration
    """
    pass