import numpy as np
from sympy import symbols, cos,sin,pi,simplify
from sympy.matrices import Matrix

def idk():
    q1,q2,q3,q4,q5,q6 = symbols('q1 q2 q3 q4 q5 q6')
    D_H_Table = [[0,0,105,q1],
                 [0,-90,0,q2],
                 [100,0,0,q3],
                 [10,-90,95,q4],
                 [10,90,95,q5],
                 [0,-90,58,q6]]
    
    T01 = get_transformation_matrix(D_H_Table[0][0],D_H_Table[0][1],D_H_Table[0][2],D_H_Table[0][3])
    T12 = get_transformation_matrix(D_H_Table[1][0],D_H_Table[1][1],D_H_Table[1][2],D_H_Table[1][3])
    T23 = get_transformation_matrix(D_H_Table[2][0],D_H_Table[2][1],D_H_Table[2][2],D_H_Table[2][3])
    T34 = get_transformation_matrix(D_H_Table[3][0],D_H_Table[3][1],D_H_Table[3][2],D_H_Table[3][3])
    T45 = get_transformation_matrix(D_H_Table[4][0],D_H_Table[4][1],D_H_Table[4][2],D_H_Table[4][3])
    T56 = get_transformation_matrix(D_H_Table[5][0],D_H_Table[5][1],D_H_Table[5][2],D_H_Table[5][3])
    
    T06 = T01 * T12 * T23 * T34 * T45 * T56
    # Provide the joint angles
    joint_angles = [np.radians(-29.79), np.radians(1.14), np.radians(-35.68), np.radians(47.98), np.radians(98.08), np.radians(57.04)]
    # Define the offset parameters. Complete the values
    offsets = [0, -90, 0, 0, 0, 0]
    # map the joint angles and offsets into the transformation matrix
    subs_dict = {q:offset + angle for q, offset, angle in zip([q1, q2, q3, q4, q5, q6], offsets, joint_angles)}



    T06 = T06.subs(subs_dict)
    end_effector_position = T06[:3, 3]

    print(T06)



def get_transformation_matrix(a, alpha, d, theta):
    M = Matrix([[cos(theta), -1 * sin(theta) ,0 ,a],
                [sin(theta)*np.cos(alpha),cos(theta)*np.cos(alpha),-1 * np.sin(alpha),-1 * np.sin(alpha)*d],
                [sin(theta)*np.sin(alpha),cos(theta)*np.sin(alpha),np.cos(alpha),np.cos(alpha)*d],
                [0,0,0,1]])
    return M

idk()
