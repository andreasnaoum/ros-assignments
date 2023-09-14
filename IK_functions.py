#! /usr/bin/env python3
import math
import numpy as np

"""
    # Andreas Naoum
    # anaoum@kth.se
"""


# Defined Values for Scara Robot
l = [0.07, 0.30, 0.35]

def scara_IK(point):
    x = point[0]
    y = point[1]
    z = point[2]
    q = [0.0, 0.0, 0.0]
    
    
    x = x - l[0]

    c2 = (pow(x,2) + pow(y,2) - pow(l[1],2) - pow(l[2],2)) / (2*l[1]*l[2])
    s2 = sqrt(1-pow(c2,2))

    k1 = l[1] + l[2]*c2
    k2 = l[2]*s2

    q[0] = atan2(y,x) - atan2(k2,k1)
    q[1] = atan2(s2, c2)
    q[2] = z
    
    return q

# Defined Values for Scara Robot
l0 = 0.311
L = 0.4
M = 0.39
l7 = 0.078


# --------------- Defined Values for Scara Robot ----------------
l0 = 0.311
L = 0.4
M = 0.39
l7 = 0.078

tolerance = 0.0005

DEBUG = False
TEST = False


# ------------------- Supportive Functions -------------------

def DH_transformation(alpha, a, d, theta):
        return np.array(
                [
                    [cos(theta), -sin(theta) * cos(alpha),  sin(theta) * sin(alpha), d * cos(theta)],
                    [sin(theta),    cos(theta)*cos(alpha), -cos(theta) * sin(alpha), d * sin(theta)],
                    [         0,               sin(alpha),               cos(alpha),              a],
                    [         0,                        0,                        0,              1]
                ]
        )


def matrix_multiply1(matrix):
        return np.dot(
          matrix,
          ([0],[0],[1])
        )


def matrix_multiply2(matrix):
        return np.dot(
          matrix,
          ([0],[0],[0],[1])
        )


def calculate_error(end_effector_transform, R, point):
    position_error = end_effector_transform[0:3,3] - point
    orientation_error = (0.5 * np.cross(end_effector_transform[:3,0], R[:, 0]) +  0.5 * np.cross(end_effector_transform[:3,1], R[:, 1]) +  0.5 * np.cross(end_effector_transform[:3,2], R[:, 2]))
    return np.concatenate((position_error, orientation_error))


# ------------------- KUKA Robot Function -------------------

"""
The inputs to this function are: 

    point = (x, y, z), the desired position of the end-effector.
    R = 3x3 rotation matrix, the desired orientation of the end-effector.
    joint_positions = (q1, q2, q3, q4, q5, q5, q7) the current joint positions.

The output of this function is a vector q containing the 7 joint values that give the desired pose of the end-effector. 

"""
def kuka_IK(point, R, joint_positions):
    # x = point[0]
    # y = point[1]
    # z = point[2]
    q = joint_positions #it must contain 7 elements

    while True:

        T_list = []
        T0_X_list = []
        rotation_list = []
        vector_list = []
        pi_list = []
        col_list = []

        DH =(
			[  (pi/2),    l0, 0, q[0]],
            [ -(pi/2),     0, 0, q[1]],
            [ -(pi/2),     L, 0, q[2]],
            [  (pi/2),     0, 0, q[3]],
            [  (pi/2),     M, 0, q[4]],
            [ -(pi/2),     0, 0, q[5]],
            [       0,    l7, 0, q[6]]
		)

        k = 0
        # Find out T0_1, T1_2, .., T6_7
        for row in DH:
              T = DH_transformation(row[0], row[1], row[2], row[3])
              T_list.append(T)
              if DEBUG:
                print("T ", k, "-", k+1, " is ", T)

        for i in range(len(T_list)):
            if i==0:
                T0_X = T_list[i]
                vec = ([0],[0],[1])
            else: 
                T0_X = np.dot(T0_X_list[i-1], T_list[i])
                vec = matrix_multiply1(rotation_list[i-1])
            rot = T0_X[0:3, 0:3]
            rotation_list.append(rot)
            pi_i = matrix_multiply2(T0_X)
            if DEBUG: 
                print("T0_", i, " = \n", T0_X)
                print("R ", i, "is ", rot)
                print("vector ", i, "is ", vec)
            T0_X_list.append(T0_X)
            vector_list.append(vec)
            pi_list.append(pi_i)

        end_effector_transform = pi_list[-1]
        
        for i in range(len(vector_list)):
              pi_i = pi_list[i]
              p_i = np.transpose(
                        np.cross(np.transpose(vector_list[i]), 
                        np.transpose(end_effector_transform[0:3] - pi_i[0:3]))
                    )
              col = np.concatenate((p_i, vector_list[i]), axis = 0)
              col_list.append(col)

        Jacobian = np.concatenate(col_list, axis = 1)
        Jacobian_inverse =  np.linalg.pinv(Jacobian)
        error_matrix = calculate_error(T0_X_list[-1], np.array(R), point)

        if TEST==True or np.linalg.norm(error_matrix) < tolerance:
             print("Out of the loop!")
             break
        else:
             q = q - np.dot(Jacobian_inverse, error_matrix)

    return q
