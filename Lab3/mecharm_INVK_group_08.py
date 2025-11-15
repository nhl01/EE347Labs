import csv
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

# With forward kinemetics, we're getting the coordinates with given angles values.... return the coordinates of end effector
def symbolic_forward_kinematics(q_values):
    # Map the joint angles and offsets into the transformation matrix
    subs_dict = {q:offset + angle for q, offset, angle in zip([q1, q2, q3, q4, q5, q6], offsets, q_values)}
    T_symbolic = T06.subs(subs_dict)
    return T_symbolic

# Turn to numpy function
forward_kinematics_func = lambdify(q_sym, symbolic_forward_kinematics(q_sym), 'numpy')

# Get the forward kinematic error for the x y z between forward kinematics with the desired target
def position_error(q_position, x_target, y_target, z_target):
    # Forward kinematics for first 3 links
    T_values = forward_kinematics_func(q_position[0], q_position[1], q_position[2], 0, 0, 0)
    # Extract X, Y, Z values
    X, Y, Z = T_values[:3, 3]
    # Return the extracted minus target positions
    return [X-x_target, Y-y_target, Z-z_target]

# Get the orientation error like roll, pitch and taw between the forward kinematic with desired target
def orientation_error(q_orientation, rx_d, ry_d, rz_d):
    T_values = forward_kinematics_func(0, 0, 0, q_orientation[0], q_orientation[1], q_orientation[2])
    # Extract X, Y, Z values
    
    roll, pitch, yaw = np.arctan2(T_values[2][1], T_values[2][2]), np.arctan2(-1*T_values[2][0], np.sqrt(T_values[0][0]*T_values[0][0]+T_values[1][0]*T_values[1][0])), np.arctan2(T_values[1][0], T_values[0][1] )
    
    return [roll - rx_d, pitch - ry_d, yaw - rz_d]
    
# Solving for 6 angles of the robot with given coordinates of the end-effector...
def inverse_kinematics(x_target, y_target, z_target, rx_d, ry_d, rz_d, q_init, max_iterations=100, tolerance=1e-6):
    position_args = (x_target, y_target, z_target)
    q_position_solution = least_squares(position_error, q_init[:3], args=position_args, method='lm', max_nfev=max_iterations, ftol=tolerance).x

    orientation_args = (rx_d, ry_d, rz_d)
    q_orientation_solution = least_squares(orientation_error, q_init[3:], args=orientation_args, method='lm', max_nfev=max_iterations, ftol=tolerance).x

    return np.concatenate((q_position_solution, q_orientation_solution))

# Reading the file and return the recorded angles and coordinates. Again, even line of the data.csv is angles and odd is the coordinates
def readCSV():
    # Load Data
    filename = "../Lab2/data.csv"
    angles = []
    coord = []

    with open(filename, mode="r") as f:
        reader = csv.reader(f)         
        i = 0
        #The csv read comes in this format: ['[11.42', ' 119.35', ' 166.72', ' 133.94', ' -81.56', ' -62.84]']
        for row in reader:
            #Remove [ and ] from first index string and last one
            print()
            row[0] = row[0].replace('[', '')
            row[-1] = row[-1].replace(']', '')
            #Conver String to float
            temp = []
            for col in row:
                temp.append(float(col))

            #even Line of the csv is angle and odd is coord here, because i is 0-based
            if (i % 2 == 0):
                angles.append(temp)
            else:
                coord.append(temp)
            i+=1
    return angles, coord    

# This function will throw the coordinates into the reverse kinmatics and get the angles; next throw that angles in forward kinematics to get the coordinates to compare with our actual
def testingWithRecordedCoor():
    # Get angles and coordinates from the csv file
    angles, coor= readCSV()
    
    for i in range(len(angles)):
        # Define the x y z and roll pitch yaw from each line of coordinates.... also get the 6 angles of robot
        x_target = coor[i][0]
        y_target = coor[i][1]
        z_target = coor[i][2]
        rx_d = np.radians(coor[i][3])  # Roll angle (in radians)
        ry_d = np.radians(coor[i][4])  # Pitch angle (in radians)
        rz_d = np.radians(coor[i][5])  # Yaw angle (in radians)
        q_init = angles[i]

        # Apply inverse kinematics with given coordinates
        joint_angles = inverse_kinematics(x_target, y_target, z_target, rx_d, ry_d, rz_d, q_init)
        # Apply forward kinematics with computed angles to check if computed coordinates is an exact match on recorded coordinates
        # Un comment the couple line below to see if computed coordinates is same exact as recorded one (we keep it comment because the deliverables want us to print angles)

        #cal_coor = symbolic_forward_kinematics(joint_angles)
        #end_effector_position = cal_coor[:3, 3]

        # Print out the computed coordinates
        #print("Coor:")
        #for j in range(len(end_effector_position)):
        #    print("c" + str(j) + ": " + str(end_effector_position[j]))

        print("Angles:")
        for j in range(len(joint_angles)):
            print("c" + str(j) + ": " + str(joint_angles[j]))

testingWithRecordedCoor() # Call this function to test the inverse kinematics


