#!/usr/bin/env python3


# Imports
import serial, time
from kinematics import Kinematics

# Robot geometry
geometry = [
    #xyz
    [0,0,109.4], # V0
    [0,0,112.6], # V1
    [57.7,0,0],  # V2
    [80.8,0,0],  # V3
    [67.4,0,0]   # V4
]


joint_limits = [
    [-90, 190],
    [-42, 110],
    [-180, 2],
    [0, 180],
    [-102, 102],
    [-90, 240],
    [0, 180]
]

# Create the kinematics model of the robot
robot = Kinematics(geometry, joint_limits)

USB_PORT = "/dev/ttyACM0"  # Arduino Mega 2560

# Connect to USB serial port at 9600 baud
try:
   usb = serial.Serial(USB_PORT, 9600, timeout=2)
except:
   print("ERROR - Could not open USB serial port.  Please check your port name and permissions.")
   print("Exiting program.")
   exit()

# List of commands
commands = ["TriggerEndstops\n", "Home\n", "MoveJoints\n"]



# Wait for the robot to finish init
while True:
    line = usb.readline()  # read input from Arduino
    line = line.decode()  # convert type from bytes to string
    line = line.strip()  # strip extra whitespace characters
    if line == "ok init":
        print("Robot arm finish init")
        break



def send_position(position, speed):
    """ Gets positons in degrees for each
    joint and sends them to the robot"""

    usb.write(commands[2].encode('UTF-8'))
    time.sleep(0.05)
    print("Sending: " + commands[2])

    # Convert positons to stringsb before encoding and sending
    for i in range(len(position)):
        position[i] = str(position[i])
        usb.write(str(position[i]+"\n").encode('UTF-8'))
        time.sleep(0.05)

    speed = str(speed)
    usb.write(str(speed+"\n").encode('UTF-8'))



def get_arduino_data():
    """ Reads the serial data from the arduino
    and checks if it is ready to recieve new informaton """

    line = usb.readline()  # read input from Arduino
    line = line.decode()  # convert type from bytes to string
    line = line.strip()  # strip extra whitespace characters
    if line != "":
        print(line)

    if line == "finish":
        print("Robotarm ready for new command")
        time.sleep(0.05)
        return True
    else:
        return False



coordinates = [
    #x,y,z a,b,c gripper
    [140, -81,   134, -3.14,  0,  -1.27, 0],
    [211, -81,   134, -3.14,  0,  -1.27, 0],
    [211,  81,   134, -3.14,  0,  -1.27, 0],
    [140,  81,   134, -3.14,  0,  -1.27, 0]
]


counter = 0

FIRST_RUN = True
# Send and recieve data forever
while True:
    # If the arduino is ready to recieve new commands
    if get_arduino_data() or FIRST_RUN:
        FIRST_RUN = False
        if counter == len(coordinates):
            break
            counter = 0
            
        angles = robot.convertJointAngleToRobot(robot.inverseKin(coordinates[counter][:-1]))
        angles.append(coordinates[counter][-1])
        send_position(angles, 1200)
        counter += 1


