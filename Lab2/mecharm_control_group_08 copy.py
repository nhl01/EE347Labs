from time import sleep
from pymycobot.mycobot import MyCobot
from pymycobot import PI_PORT, PI_BAUD
import math

# Display the current end effector coordinates and the corresponding joint angles
def displayRobotStatus(mycobot: MyCobot):
    angles = mycobot.get_angles()
    coord = mycobot.get_coords()
    print("Current angles: ", angles)
    #x, y, z, rx, ry, rz
    print("Current Coordinates: ", coord)

def main():
    mycobot = MyCobot(PI_PORT, PI_BAUD)
    mycobot.power_on()
    mycobot.release_all_servos()
    # Print the status of the robotic arm for 11 times (1st input is always wrong)
    for i in range(11):
        print("Position: ", i)
        displayRobotStatus(mycobot)
        print("\n")
        sleep(10)
    mycobot.power_off()

main()