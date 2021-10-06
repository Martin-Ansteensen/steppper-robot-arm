
#!/usr/bin/env python3

from os import wait
import serial, time
from serial.serialutil import SerialException

class ArduinoCommunication():
    def __init__(self) -> None:
        self.ready = False
        self.send_delay = 0.04
        # Connect to USB serial port at 9600 baud
        USB_PORT = "/dev/ttyACM0"  # Arduino Mega 2560
        failed_tries = 0
        while True:
            try:
                # Changing the timeout affects the runtime a lot!
                self.usb = serial.Serial(USB_PORT, 9600, timeout=0.1)
                print("Connection successful")
                break
            except SerialException:
                if failed_tries == 0:
                    print("ERROR - Could not open USB serial port.  Please check your port name and permissions.")
                failed_tries = 1
                # print("Exiting program.")
                # exit()

        # Wait for the robot to finish init
        while True:
            line = self.usb.readline()  # read input from Arduino
            line = line.decode()  # convert type from bytes to string
            line = line.strip()  # strip extra whitespace characters
            if line == "ok init":
                print("Robot arm finish init")
                self.ready = True
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
            time.sleep(self.send_delay)

        speed = str(speed)
        self.usb.write(str(speed+"\n").encode('UTF-8'))
        self.wait_for_confirmation()

    def send_MoveJoint(self, joint, angle, speed):
        """ Sends each joints angle to the robot
        with a MoveJoints command """
        command = "MoveJoint\n"
        self.usb.write(command.encode('UTF-8'))
        time.sleep(self.send_delay)
        print("Sending: " + command)

        data = [joint, angle, speed]

        # Convert positons to stringsb before encoding and sending
        for i in range(len(data)):
            data[i] = str(data[i])
            self.usb.write(str(data[i]+"\n").encode('UTF-8'))
            time.sleep(0.02)
        self.wait_for_confirmation()        

    def send_Home(self):
        """ Sends the Home command to go
        to the home position relative to
        the endstops. Must only be used after
        a TriggerEndstops command """

        command = "Home\n"
        self.usb.write(command.encode('UTF-8'))
        time.sleep(0.02)
        print("Sending: " + command)
        self.wait_for_confirmation()

    def send_TriggerEndstops(self):
        """ Sends the TriggerEndstops command
        to zero all joints """

        command = "TriggerEndstops\n"
        self.usb.write(command.encode('UTF-8'))
        time.sleep(0.02)
        print("Sending: " + command)
        self.wait_for_confirmation()

    def read_serial(self):
        """ Reads the serial data (only one line) from the arduino
        and checks if it is ready to recieve new informaton """

        line = self.usb.readline()  # read input from Arduino
        line = line.decode()  # convert type from bytes to string
        line = line.strip()  # strip extra whitespace characters
        if not line:  
            # The line is empty -> the arm does not have 
            # anything to say
            return False

        # Print the line since it is not empty
        print("Arduino: " + line)

        if line == "ready":
            self.ready = True
        elif line == "busy":
            self.ready = False
        # Return True because the arm had something
        # meaningful to say
        return line
    
    def wait_for_confirmation(self):
        """ Read the serial until the arm either confirms that
        the command is being executed (busy) or that it quit 
        due to some error (quit) """

        while True:
            line = self.read_serial()
            if line == "busy" or line == "quit":
                break

    def wait_for_ready(self, wait_command):
        """ Waits until it recieves a ready
        from the arduino """
        while not self.ready:
            self.read_serial()
            wait_command()