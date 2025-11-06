import numpy as np
from sympy  import symbols, cos, sin, atan2, pi, Matrix, lambdify 
from scipy.optimize import least_squares

# Declare symbols
q1,q2,q3,q4,q5,q6 = symbols('q1 q2 q3 q4 q5 q6')

# Define the offset parameters
offsets = [0, 0, 0, 0, 0, 0]

# Declare the DH Table
D_H_Table = [[0,0,114,q1],
            [0,-90,0,q2-pi/2],
            [100,0,0,q3],
            [10,-90,95,q4],
            [0,90,0,q5],
            [0,-90,58,q6]]

# Helper function to take the D-H table row values as input and outputs the corresponding transformation matrix
def get_transformation_matrix(a, alpha, d, theta):
    # Convert our alpha from degree to radian
    alpha = np.radians(alpha)
    M = Matrix([[cos(theta), -1 * sin(theta), 0, a],
                [sin(theta)*np.cos(alpha),cos(theta)*np.cos(alpha),-1 * np.sin(alpha),-1 * np.sin(alpha)*d],
                [sin(theta)*np.sin(alpha),cos(theta)*np.sin(alpha),np.cos(alpha),np.cos(alpha)*d],
                [0,0,0,1]])
    return M

# Calculate overall transformation matrix
T06 = get_transformation_matrix(D_H_Table[0][0],D_H_Table[0][1],D_H_Table[0][2],D_H_Table[0][3])
for i in range(1, len(D_H_Table)):
    T06 = T06 @ get_transformation_matrix(D_H_Table[i][0], D_H_Table[i][1], D_H_Table[i][2], D_H_Table[i][3])

# Define symbolic joint variables for differentiation
q_sym = symbols('q1:7')

def symbolic_forward_kinematics(q_values):
    # Map the joint angles and offsets into the transformation matrix
    subs_dict = {q:offset + angle for q, offset, angle in zip([q1, q2, q3, q4, q5, q6], offsets, q_values)}
    T_symbolic = T06.subs(subs_dict)
    return T_symbolic

forward_kinematics_func = lambdify(q_sym, symbolic_forward_kinematics(q_sym), 'numpy')

def position_error(q_position, x_target, y_target, z_target):
    # Forward kinematics for first 3 links
    T_values = forward_kinematics_func(q_position[1], q_position[2], q_position[3], 0, 0, 0)
    # Extract X, Y, Z values
    X, Y, Z = T_values[:3, 3]
    # Return the extracted minus target positions
    return [X-x_target, Y-y_target, Z-z_target]

def orientation_error(q_orientation, rx_d, ry_d, rz_d):
    T_values = forward_kinematics_func(0, 0, 0, q_orientation[4], q_orientation[5], q_orientation[6])
    # Extract X, Y, Z values
    X, Y, Z = T_values[:3, 3]
    roll, pitch, yaw = np.arctan2(Z,Y), np.arctan2(-X,np.sqrt(Y*Y+Z*Z)), np.arctan2(Y,X)
    
    return [roll - rx_d, pitch - ry_d, yaw - rz_d]
    
print(forward_kinematics_func(0, 0, 0, 0, 0, 0))

def inverse_kinematics(x_target, y_target, z_target, rx_d, ry_d, rz_d, q_init, link_lengths, max_iterations=100, tolerance=1e-6):
    position_args = (x_target, y_target, z_target, link_lengths)
    q_position_solution = least_squares(position_error(q_init, x_target, y_target, z_target), q_init[:3], args=position_args, method='lm', max_nfev=max_iterations, ftol=tolerance).x

    orientation_args = (rx_d, ry_d, rz_d)
    q_orientation_solution = least_squares(orientation_error, q_init[3:], args=orientation_args, method='lm', max_nfev=max_iterations, ftol=tolerance).x

    return np.concatenate(q_position_solution, q_orientation_solution)