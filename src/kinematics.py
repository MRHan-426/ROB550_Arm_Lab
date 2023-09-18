"""!
Implements Forward and Inverse kinematics with DH parametrs and product of exponentials

TODO: Here is where you will write all of your kinematics functions
There are some functions to start with, you may need to implement a few more
"""

import numpy as np
import math
from scipy.linalg import expm
from resource.config_parse import parse_dh_param_file, parse_pox_param_file
import matplotlib.pyplot as plt
import csv

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


def FK_dh(dh_params, joint_angles, link = 5):
    """!
    @brief      Get the 4x4 transformation matrix from link to world

                TODO(DONE): implement this function

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

    TODO(DONE): Find the T matrix from a row of a DH table

    @param      a      a meters
    @param      alpha  alpha radians
    @param      d      d meters
    @param      theta  theta radians

    @return     The 4x4 transformation matrix.
    """
    A_i = np.array([[np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha) , a * np.cos(theta)],
                    [np.sin(theta), np.cos(theta)*np.cos(alpha) , -np.cos(theta)*np.sin(alpha), a * np.sin(theta)],
                    [0            , np.sin(alpha)               , np.cos(alpha)               , d                ],
                    [0            , 0                           , 0                           , 1                ]])
    return A_i


def get_euler_angles_from_T(T, style = "zyz"):
    """!
    @brief      Gets the euler angles from a transformation matrix.

                TODO(DONE): Implement this function return the 3 Euler angles from a 4x4 transformation matrix T
                If you like, add an argument to specify the Euler angles used (xyx, zyz, etc.)

    @param      T     transformation matrix

    @return     The euler angles from T.
    """
    R = T[0:3,0:3]

    def zyz(): # BY DEFAULT
        alpha = np.arctan2(R[1, 2], R[0, 2])
        beta = np.arctan2(np.sqrt(R[0, 2]**2 + R[1, 2]**2), R[2, 2])
        gamma = np.arctan2(R[2, 1], -R[2, 0])
        return np.array([alpha, beta, gamma])

    def xyx():
        alpha = np.arctan2(R[1, 0], -R[2, 0])
        beta = np.arctan2(np.sqrt(R[0, 0]**2 + R[0, 1]**2), R[0, 2])
        gamma = np.arctan2(R[0, 1], R[0, 0])
        return np.array([alpha, beta, gamma])

    def zyx(): 
        yaw = np.arctan2(R[1, 0], R[0, 0])
        pitch = np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))
        roll = np.arctan2(R[2, 1], R[2, 2])
        return np.array([yaw, pitch, roll])

    switcher = {
        "zyz": zyz,
        "xyx": xyx,
        "zyx": zyx}

    return switcher.get(style, zyz)()


def get_pose_from_T(T, style = "zyz"):
    """!
    @brief      Gets the pose from T.

                TODO(DONE): implement this function return the 6DOF pose vector from a 4x4 transformation matrix T

    @param      T     transformation matrix

    @return     The pose vector from T.
    """
    positions = np.array(T[0:3, 3])
    euler_angles = get_euler_angles_from_T(T, style)
    return np.concatenate((positions, euler_angles))



def FK_pox(joint_angles, m_mat, s_lst):
    """!
    @brief      Get a  representing the pose of the desired link

                TODO(DONE): implement this function, Calculate forward kinematics for rexarm using product of exponential
                formulation return a 4x4 homogeneous matrix representing the pose of the desired link

    @param      joint_angles  The joint angles
                m_mat         The M matrix
                s_lst         List of screw vectors

    @return     a 4x4 homogeneous matrix representing the pose of the desired link
    """
    T = np.eye(4)
    for i in range(len(s_lst)):
        w = s_lst[i][:3]
        v = s_lst[i][3:]
        s = to_s_matrix(w, v)
        e_s = expm(s * joint_angles[i])
        T = np.matmul(T, e_s)
        
    return np.matmul(T, m_mat)


def to_s_matrix(w, v):
    """!
    @brief      Convert to s matrix.

    TODO(DONE): implement this function
    Find the [s] matrix for the POX method e^([s]*theta)

    @param      w     { parameter_description }
    @param      v     { parameter_description }

    @return     { description_of_the_return_value }
    """
    w1,w2,w3 = w
    v1,v2,v3 = v
    s = np.array([[0,-w3,w2,v1],
                [w3,0,-w1,v2],
                [-w2,w1,0,v3],
                [0,0,0,0]])
    return s


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


def plot_joint_angles(file_path):
    """!
    """
    x,y1,y2,y3,y4,y5 = [],[],[],[],[],[]

    with open(file_path, mode='r', newline='') as file:
        reader = csv.reader(file)
        for row in reader:
            x.append(float(row[0]))
            y1.append(float(row[1]))
            y2.append(float(row[2]))
            y3.append(float(row[3]))
            y4.append(float(row[4]))
            y5.append(float(row[5]))
            
    fig,ax = plt.subplots()
    ax.plot(x, y1,label='Joint1')
    ax.plot(x, y2,label='Joint2')
    ax.plot(x, y3,label='Joint3')
    ax.plot(x, y4,label='Joint4')
    ax.plot(x, y5,label='Joint5')
    ax.legend()
    ax.set_xlabel('Time')
    ax.set_ylabel('Joint Angle (radians)')
    ax.set_title('Joint Angles Over Time for 1 Cycle')
    plt.grid(True)
    plt.show()

# For test of Forward Kinematics
if __name__ == '__main__':
    dh_config_file = "config/rx200_dh.csv"
    dh_params = parse_dh_param_file(dh_config_file)
    pox_config_file = "config/rx200_pox.csv"
    m_mat, s_lst = parse_pox_param_file(pox_config_file)
    # -0.73170888  0.320602    0.53689331 -0.0076699   0.01073787 
    joint_angles = [clamp(-0.73170888),clamp(0.320602),clamp(0.53689331),clamp(-0.0076699),clamp(0.01073787)]

    T_DH = FK_dh(dh_params=dh_params, joint_angles=joint_angles, link=5)
    T_POX = FK_pox(joint_angles=joint_angles, m_mat=m_mat,s_lst=s_lst)

    if np.allclose(T_DH, T_POX, rtol=1e-05, atol=1e-08):
        print("POX and DH get the same result, mission complete.")
    else:
        print("ERROR OCCURS")
        print(f"DH Transform Matrix:\n {T_DH}")
        print(f"POX Transform Matrix:\n {T_POX}")
    # file_path = "example.csv"
    # plot_joint_angles(file_path)