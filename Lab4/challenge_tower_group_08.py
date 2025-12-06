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

# perform a pick and drop action with the corresponding coordinates
def sendRobotToSpecificLocation(angles, coords, readyPick, readyDrop):
    # Initial our robot arm
    mycobot = MyCobot(PI_PORT, PI_BAUD)
    mycobot.power_on()

    # Set robot to default position with the gripper open
    sleep(2)
    mycobot.send_angles([0,0,0,0,0,0],40)
    print("Start coordinates")
    sleep(2)
    mycobot.set_gripper_state(0,40)
    sleep(1)

    # There'll be two states
    # states 1: pick up states
    # states 2: drop off states
    states = 1

    # blockPick and blockDrop act as an index for the readyPick and readyDrop array
    blockPick = 0
    blockDrop = 0

    # Loop through all the coordinates
    for i in range(len(coords)):
        if (states == 1):
            # end effector will move to a specific via point before picking a corresponding block
            mycobot.send_coords(getCoordinates(readyPick[blockPick][0], mycobot.get_angles()), 30)
            print("Go to ready to pick states")
            sleep(2)

        elif (states == 2):
            # end effector will move to a specific via point before droping a corresponding block
            mycobot.send_coords(getCoordinates(readyDrop[blockDrop][0], mycobot.get_angles()), 30)
            print("Go to ready to drop states")
            sleep(2)

        # end effector will move to the block/location of block. A place where end effector can drop/pick up the block
        end_effector_position = getCoordinates(coords[i], mycobot.get_angles())
        mycobot.send_coords(end_effector_position, 20)
        print("Go to pick/drop states")
        sleep(3)

        if (states == 1):
            # end effector will close to grab the block
            mycobot.set_gripper_value(70,30,1)
            print("pick up")
            sleep(2)
            # back to the previous via point
            mycobot.send_coords(getCoordinates(readyPick[blockPick][0], mycobot.get_angles()), 50)
            # Next time robot will move to a different via point to pick up another block
            blockPick+=1
            print("Go to ready to pick states")
            sleep(2)
            # Next time robot should perform drop block
            states = 2
        elif (states == 2):
            # end effector will open to drop the block
            mycobot.set_gripper_state(0,20)
            print("Drop")
            sleep(2)
            # back to the previous via point
            mycobot.send_coords(getCoordinates(readyDrop[blockDrop][0], mycobot.get_angles()), 30)
            # Next time robot will move to a different via point to drop another block
            blockDrop+=1
            print("Go to ready to drop states")
            sleep(2)
            # Next time robot should perform pick up block
            states = 1

        # Back to default position
        print("Go to Neutral States")
        mycobot.send_angles([0,0,0,0,0,0],30)
        sleep(2)


#Performs inverse kinematics to get solution joint angles, and recalculates solution coordinates using our own forward kinematics
# return the calculated coordinates
def getCoordinates(coords, angles):
    x_target = coords[0]
    y_target = coords[1]
    z_target = coords[2]
    rx_d = np.radians(coords[3])  # Roll angle (in radians)
    ry_d = np.radians(coords[4])  # Pitch angle (in radians)
    rz_d = np.radians(coords[5])  # Yaw angle (in radians)
    q_init = angles
    joint_angles = ik((x_target, y_target, z_target, rx_d, ry_d, rz_d), q_init) # apply inverse kinematic to find computed angles
    calculated_coords = symbolic_forward_kinematics(joint_angles) # apply forward kinematics to find the computed coordinates with computed angles
    # Since the coordinates that we got only contains x y z... we then add the "recorded" rx ry rz
    end_effector_position = calculated_coords[:3, 3][0:3] + [coords[3], coords[4], coords[5]]
    # Send coordinates to robot
    return end_effector_position

# Main function to break the data in csv file to angles and coordinates; also break down the via point to pick up/drop
# The block.csv file is structured with even line(0...2...4) to be angles and odd line(1...3...5) to be coordinates
# angles from the csv will not be used....
# First 6 line of the csv is a via point before picking up block 1, 3, 5.... next 6 line is via point before dropping block 1, 3, 5
# The rest of the csv is position to pick up/drop the block. it goes by position to pick up... drop.... pick up... drop

def blockPickNDrop():
    angles,coords = readCSV("block.csv")
    # extra the via point before pick up and drop
    readyPick = [[coords[0],angles[0]],[coords[1],angles[1]],[coords[2],angles[2]]]
    readyDrop = [[coords[3],angles[3]],[coords[4],angles[4]],[coords[5],angles[5]]]

    # The rest of the point will be position to pick up and drop... send those along with readyPick and readyDrop
    sendRobotToSpecificLocation(angles[6:], coords[6:], readyPick,readyDrop)

# Testing if robot can move from base to desired starting position and to desired placing/picking position
def test():
    # Set up robot
    mycobot = MyCobot(PI_PORT, PI_BAUD)
    mycobot.power_on()
    sleep(2)
    # Back to base
    mycobot.send_angles([0,0,0,0,0,0],30)
    sleep(2)
    # A via point
    mycobot.send_coords([165.8, -67.0, 150.4, 111.45, 76.79, 131.24],20)
    sleep(2)
    # Pick or Place position
    mycobot.send_coords([165.8, -67.0, 110.4, 111.45, 76.79, 131.24],20)

    
    sleep(2)
    
#Read10Position() #Uncomment this to drag teach

#blockPickNDrop() #Uncomment to perform pick and drop block from one loading zone to another.... final demo for challenge 2

#test() # Uncomment this to test if robotic arm can move to desired positition
