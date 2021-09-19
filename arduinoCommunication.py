
#!/usr/bin/env python3

import serial, time
from serial.serialutil import SerialException

class ArduinoCommunication():
    def __init__(self) -> None:

        # Connect to USB serial port at 9600 baud
        USB_PORT = "/dev/ttyACM0"  # Arduino Mega 2560
        try:
            self.usb = serial.Serial(USB_PORT, 9600, timeout=2)
        except SerialException:
            print("ERROR - Could not open USB serial port.  Please check your port name and permissions.")
            print("Exiting program.")
            exit()

        # Wait for the robot to finish init
        while True:
            line = self.usb.readline()  # read input from Arduino
            line = line.decode()  # convert type from bytes to string
            line = line.strip()  # strip extra whitespace characters
            if line == "ok init":
                print("Robot arm finish init")
                break

        # List of commands
        self.commands = ["TriggerEndstops\n", "Home\n", "MoveJoints\n"]

    def send_MoveJoints(self, position, speed):
        """ Sends each joints angle to the robot
        with a MoveJoints command """
        command = "MoveJoints\n"
        self.usb.write(command.encode('UTF-8'))
        time.sleep(0.02)
        print("Sending: " + command)

        # Convert positons to stringsb before encoding and sending
        for i in range(len(position)):
            position[i] = str(position[i])
            self.usb.write(str(position[i]+"\n").encode('UTF-8'))
            time.sleep(0.02)

        speed = str(speed)
        self.usb.write(str(speed+"\n").encode('UTF-8'))

    def send_MoveJoint(self, joint, angle, speed):
        """ Sends each joints angle to the robot
        with a MoveJoints command """
        command = "MoveJoint\n"
        self.usb.write(command.encode('UTF-8'))
        time.sleep(0.02)
        print("Sending: " + command)

        data = [joint, angle, speed]

        # Convert positons to stringsb before encoding and sending
        for i in range(len(data)):
            data[i] = str(data[i])
            self.usb.write(str(data[i]+"\n").encode('UTF-8'))
            time.sleep(0.02)

    def send_Home(self):
        """ Sends the Home command to go
        to the home position relative to
        the endstops. Must only be used after
        a TriggerEndstops command """

        command = "Home\n"
        self.usb.write(command.encode('UTF-8'))
        time.sleep(0.02)
        print("Sending: " + command)

    def send_TriggerEndstops(self):
        """ Sends the TriggerEndstops command
        to zero all joints """

        command = "TriggerEndstops\n"
        self.usb.write(command.encode('UTF-8'))
        time.sleep(0.02)
        print("Sending: " + command)
 
    def read_serial(self):
        """ Reads the serial data from the arduino
        and checks if it is ready to recieve new informaton """

        line = self.usb.readline()  # read input from Arduino
        line = line.decode()  # convert type from bytes to string
        line = line.strip()  # strip extra whitespace characters
        if line != "":  # print the line if it is not empty
            print(line)

        if line == "finish":
            print("Robotarm ready for new command")
            time.sleep(0.02)
            return True
        else:
            return False
