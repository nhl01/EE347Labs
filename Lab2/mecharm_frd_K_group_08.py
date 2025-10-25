import numpy as np
from sympy import symbols, cos,sin,pi,simplify
from sympy.matrices import Matrix
from time import sleep

from pymycobot.mycobot import MyCobot
from pymycobot import PI_PORT, PI_BAUD
import math
import csv
import sympy as sp
import mecharm_control_group_08

# Compute the end-effector pose of the robot with DH table and specific set of joint angles
# input where a list of angle and output x, y, z
def applyForWardKinematic(theta):

    # Declare Variable
    q1,q2,q3,q4,q5,q6 = symbols('q1 q2 q3 q4 q5 q6')

    # Declare the DH Table
    D_H_Table = [[0,0,114,q1],
                 [0,-90,0,q2-pi/2],
                 [100,0,0,q3],
                 [10,-90,95,q4],
                 [0,90,0,q5],
                 [0,-90,58,q6]]
    
    # Get the transformation matrix
    T01 = get_transformation_matrix(D_H_Table[0][0],D_H_Table[0][1],D_H_Table[0][2],D_H_Table[0][3])
    T12 = get_transformation_matrix(D_H_Table[1][0],D_H_Table[1][1],D_H_Table[1][2],D_H_Table[1][3])
    T23 = get_transformation_matrix(D_H_Table[2][0],D_H_Table[2][1],D_H_Table[2][2],D_H_Table[2][3])
    T34 = get_transformation_matrix(D_H_Table[3][0],D_H_Table[3][1],D_H_Table[3][2],D_H_Table[3][3])
    T45 = get_transformation_matrix(D_H_Table[4][0],D_H_Table[4][1],D_H_Table[4][2],D_H_Table[4][3])
    T56 = get_transformation_matrix(D_H_Table[5][0],D_H_Table[5][1],D_H_Table[5][2],D_H_Table[5][3])
    
    # Get the transformation matrix from joint/frame 0 to 6
    T06 = T01 @ T12 @ T23 @ T34 @ T45 @ T56
    # Provide the joint angles
    joint_angles = [np.radians(theta[0]), np.radians(theta[1]), np.radians(theta[2]), np.radians(theta[3]), np.radians(theta[4]), np.radians(theta[5])]
    # Define the offset parameters. Complete the values
    offsets = [0, 0, 0, 0, 0, 0]
    # map the joint angles and offsets into the transformation matrix
    subs_dict = {q:offset + angle for q, offset, angle in zip([q1, q2, q3, q4, q5, q6], offsets, joint_angles)}


    # Replace the symbolic joint variables
    T06 = T06.subs(subs_dict)
    
    # Display the simplified 4x4 homogeneous transformation matrix
    T06Simplify = simplify(T06)
    # Get the x, y, and z from the 4x4 matrix
    end_effector_position = T06[:3, 3]

    return end_effector_position[0:3]


# Helper function to take the D-H table row values as input and outputs the corresponding transformation matrix
def get_transformation_matrix(a, alpha, d, theta):
    # COnvert our alpha from degree to radian
    alpha = np.radians(alpha)
    M = Matrix([[cos(theta), -1 * sin(theta) ,0 ,a],
                [sin(theta)*np.cos(alpha),cos(theta)*np.cos(alpha),-1 * np.sin(alpha),-1 * np.sin(alpha)*d],
                [sin(theta)*np.sin(alpha),cos(theta)*np.sin(alpha),np.cos(alpha),np.cos(alpha)*d],
                [0,0,0,1]])
    return M

#Call this function to set position according to the coord x, y, z calculated with DH table + rx, ry, and rz from csv
def setPositionFCoordDH():
    mycobot = MyCobot(PI_PORT, PI_BAUD)
    mycobot.power_on()
    #Get angles
    angle,coord = mecharm_control_group_08.readCSV()
    #back to 0 position

    
    mycobot.send_angles([0,0,0,0,0,0],30)
    print("Start coordinates")
    sleep(3)
    mycobot.set_gripper_state(1,30)
    sleep(3)

    for i in range(5): #Go to each set of coordinates, 5 times
        print("Starting sequence iteration ", i+1)
        for j in range(len(coord)): # Length of the coord and angle shall be the same
            # Get the coordinates in the method of applying forward kinematic and rx, ry, rz are the same value that's captured
            coordinatesDHGenerate = applyForWardKinematic(angle[j]) + [coord[j][3], coord[j][4], coord[j][5]]
            print(coordinatesDHGenerate)
            # Send Coord to bot
            mycobot.send_coords(coordinatesDHGenerate,30)
            sleep(5)
            # Back to origin
            mycobot.send_angles([0,0,0,0,0,0],50)
            sleep(3)

    mycobot.power_off()

# setPositionFCoordDH() #Uncomment this to test computed coordinates...If you were running this function make sure the other file did not intended to call and run other function