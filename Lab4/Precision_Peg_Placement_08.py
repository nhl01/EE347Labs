import sys
import os

# With sys Get the path of the project root which is the EE 347 Labs folder where it hold folder of Lab2 and Lab3 and Lab4... Citation: https://stackoverflow.com/questions/21005822/what-does-os-path-abspathos-path-joinos-path-dirname-file-os-path-pardir 
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(ROOT_DIR)

# Import function from Lab2 and 3
from Lab3.mecharm_INVK_advanced_group_08 import *   # or specific functions
from Lab2.mecharm_control_group_08 import Read10Position

import csv
from pymycobot.mycobot import MyCobot
from pymycobot import PI_PORT, PI_BAUD


def sendRobotToSpecificLocation(filename):
    # Initial our robot arm
    mycobot = MyCobot(PI_PORT, PI_BAUD)
    mycobot.power_on()

    # Get angles
    angles,coords = readCSV(filename)

    # Set robot to default position with the gripper open
    sleep(3)
    mycobot.send_angles([0,0,0,0,0,0],30)
    print("Start coordinates")
    sleep(3)
    mycobot.set_gripper_state(0,30)
    sleep(3)

    movedTime = 1

    for i in range(len(coords)):
        # Find compute angles with inverse kinematics and then use forward kinematics to find the coordinates position at starting position(where the marker is at)
        x_target = coords[i][0]
        y_target = coords[i][1]
        z_target = coords[i][2]
        rx_d = np.radians(coords[i][3])  # Roll angle (in radians)
        ry_d = np.radians(coords[i][4])  # Pitch angle (in radians)
        rz_d = np.radians(coords[i][5])  # Yaw angle (in radians)
        q_init = angles[i]
        joint_angles = ik((x_target, y_target, z_target, rx_d, ry_d, rz_d), q_init) # apply inverse kinematic to find computed angles
        calculated_coords = symbolic_forward_kinematics(joint_angles) # apply forward kinematics to find the computed coordinates with computed angles
        # Since the coordinates that we got only contains x y z... we then add the "recorded" rx ry rz
        end_effector_position = calculated_coords[:3, 3][0:3] + [coords[i][3], coords[i][4], coords[i][5]]
        # Send coordinates to robot
        mycobot.send_coords(end_effector_position, 10)
        print("Move Grapper")
        sleep(5)
        
        # I assume that robot will move a same number of time to the place where we could drop a block or pick up one
        match(movedTime):
            case ??:
                mycobot.set_gripper_state(1,30)
                sleep(5)
                movedTime = 0 # Reset TIme
        
            case ??:
                mycobot.set_gripper_state(0,30)
                sleep(5)
                movedTime = 0 # Reset TIme

        # View the current coordinate we at
        print("Coor:")
        for j in range(len(end_effector_position)):
            print("c" + str(j) + ": " + str(end_effector_position[j]))
        # Back to neutral
        print("Neutral States")
        mycobot.send_angles([0,0,0,0,0,0],30)

        movedTime += 1 # There must be couple helper states that we must go through before we pick up or drop

#Read10Position() #Uncomment this to drag teach