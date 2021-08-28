#!/usr/bin/env python3


# Imports
import serial, time


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

# Waiu for the robot to finish init
while True:
    line = usb.readline()  # read input from Arduino
    line = line.decode()  # convert type from bytes to string
    line = line.strip()  # strip extra whitespace characters
    if line == "ok init":
        print("Robot arm finish init")
        break

ready_for_new_command = True
counter = 0

positions = [
    [45,  90,  85, 90,  90, 0, 160],
    [0,   0,  -85, 0,   90, 0, 0],
    [-45, 30, -35, 90, -90, 0, 160]
]

for l in range(len(positions)):
    for i in range(len(positions[l])):
        positions[l][i] = str(positions[l][i])

while True:
    if ready_for_new_command:
        if counter == 3:
            counter = 0
        print("Sending new command to robot arm")
        usb.write(commands[2].encode('UTF-8'))
        time.sleep(0.05)

        for i in positions[counter]:
            usb.write(str(i+"\n").encode('UTF-8'))
            time.sleep(0.05)
        
        usb.write(b'1000\n')
        counter += 1
        ready_for_new_command = False
        # if counter == 0:
        #     print(commands[0])
        #     usb.write(commands[0].encode('UTF-8'))
        #     counter = 1
        # else:
        #     print(commands[1])
        #     usb.write(commands[1].encode('UTF-8'))
        #     counter = 0
        # ready_for_new_command = False

    line = usb.readline()  # read input from Arduino
    line = line.decode()  # convert type from bytes to string
    line = line.strip()  # strip extra whitespace characters
    print(line)

    if line == "finish" and not ready_for_new_command:
        ready_for_new_command = True
        print("Robotarm ready for new command")
        time.sleep(0.1)
