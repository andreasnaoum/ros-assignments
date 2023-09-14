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

tolerance = 0.5


# ------------------- Supportive Functions -------------------

def DH_transformation(alpha, a, d, q):
        return np.array(
                [
                    [cos(q), -sin(q) * cos(alpha),  sin(q) * sin(alpha), d * cos(q)],
                    [sin(q),    cos(q)*cos(alpha), -cos(q) * sin(alpha), d * sin(q)],
                    [     0,           sin(alpha),           cos(alpha),          a],
                    [     0,                    0,                    0,          1]
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


def calculate_error(T0_7, R, point):
    error_angle = 0.5*( np.cross(T0_7[[0,1,2],0], np.array([R[0][0], R[1][0], R[2][0]])) + np.cross(T0_7[[0,1,2],1], np.array([R[0][1], R[1][1], R[2][1]])) + np.cross(T0_7[[0,1,2],2], np.array([R[0][2], R[1][2], R[2][2]])) )
    error_position = T0_7[[0,1,2],3] - point
    error = np.array([error_position[0], error_position[1], error_position[2], error_angle[0], error_angle[1], error_angle[2]])
    return error


# ------------------- KUKA Robot Function -------------------


def kuka_IK(point, R, joint_positions):
    x = point[0]
    y = point[1]
    z = point[2]
    q = joint_positions #it must contain 7 elements

    error = 1
    while error > 0.0001:

        DH =(
			[  (pi/2), l0, 0, q[0]],
            [ -(pi/2),     0, 0, q[1]],
            [ -(pi/2),     L, 0, q[2]],
            [  (pi/2),     0, 0, q[3]],
            [  (pi/2),     M, 0, q[4]],
            [ -(pi/2),     0, 0, q[5]],
            [       0, l7, 0, q[6]]
		)

        T0_1 = DH_transformation(DH[0][0],DH[0][1],DH[0][2],DH[0][3])
        T1_2 = DH_transformation(DH[1][0],DH[1][1],DH[1][2],DH[1][3])
        T2_3 = DH_transformation(DH[2][0],DH[2][1],DH[2][2],DH[2][3])
        T3_4 = DH_transformation(DH[3][0],DH[3][1],DH[3][2],DH[3][3])
        T4_5 = DH_transformation(DH[4][0],DH[4][1],DH[4][2],DH[4][3])
        T5_6 = DH_transformation(DH[5][0],DH[5][1],DH[5][2],DH[5][3])
        T6_7 = DH_transformation(DH[6][0],DH[6][1],DH[6][2],DH[6][3])

        T0_2 = np.dot(T0_1, T1_2)
        T0_3 = np.dot(T0_2, T2_3) 
        T0_4 = np.dot(T0_3, T3_4)
        T0_5 = np.dot(T0_4, T4_5)
        T0_6 = np.dot(T0_5, T5_6)
        T0_7 = np.dot(T0_6, T6_7)

        R1 = T0_1[0:3, 0:3]
        R2 = T0_2[0:3, 0:3]
        R3 = T0_3[0:3, 0:3]
        R4 = T0_4[0:3, 0:3]
        R5 = T0_5[0:3, 0:3]
        R6 = T0_6[0:3, 0:3]
        # R7 = T0_7[0:3, 0:3]

        z1 = ([0],[0],[1])
        z2 = matrix_multiply1(R1)
        z3 = matrix_multiply1(R2)
        z4 = matrix_multiply1(R3)
        z5 = matrix_multiply1(R4)
        z6 = matrix_multiply1(R5)
        z7 = matrix_multiply1(R6)

        Pi1 = matrix_multiply2(T0_1)
        Pi2 = matrix_multiply2(T0_2)
        Pi3 = matrix_multiply2(T0_3)
        Pi4 = matrix_multiply2(T0_4)
        Pi5 = matrix_multiply2(T0_5)
        Pi6 = matrix_multiply2(T0_6)
        Pi7 = matrix_multiply2(T0_7)

        P01 = np.transpose(np.cross(np.transpose(z1), np.transpose(Pi7[0:3] - Pi1[0:3])))
        Pcol1 = np.concatenate((P01,z1), axis = 0)

        P02 = np.transpose(np.cross(np.transpose(z2), np.transpose(Pi7[0:3] - Pi2[0:3])))
        Pcol2 = np.concatenate((P02,z2), axis = 0)

        P03 = np.transpose(np.cross(np.transpose(z3), np.transpose(Pi7[0:3] - Pi3[0:3])))
        Pcol3 = np.concatenate((P03,z3), axis = 0)

        P04 = np.transpose(np.cross(np.transpose(z4), np.transpose(Pi7[0:3] - Pi4[0:3])))
        Pcol4 = np.concatenate((P04,z4), axis = 0)

        P05 = np.transpose(np.cross(np.transpose(z5), np.transpose(Pi7[0:3] - Pi5[0:3])))
        Pcol5 = np.concatenate((P05,z5), axis = 0)

        P06 = np.transpose(np.cross(np.transpose(z6), np.transpose(Pi7[0:3] - Pi6[0:3])))
        Pcol6 = np.concatenate((P06,z6), axis = 0)

        P07 = np.transpose(np.cross(np.transpose(z7), np.transpose(Pi7[0:3] - Pi7[0:3])))
        Pcol7 = np.concatenate((P07,z7), axis = 0)

        Jacob = np.concatenate((Pcol1,Pcol2,Pcol3,Pcol4,Pcol5,Pcol6,Pcol7), axis = 1)
        Jac_new =  np.linalg.pinv(Jacob)

        error_matrix = calculate_error(T0_7, R, point)
        
        error = np.linalg.norm(error_matrix)

        q = q - np.dot(Jac_new, error_matrix )

    return q

