#!/usr/bin/env python3

# Imports
from kinematics import Kinematics
from arduinoCommunication import ArduinoCommunication
from getVideoFeed import CameraInput, ShapeDetection
import cv2, atexit
# Robot geometry
geometry = [
    #xyz
    [0,0,109.4], # V0
    [0,0,112.6], # V1
    [57.7,0,0],  # V2
    [80.8,0,0],  # V3
    [130,0,0]   # V4
]

joint_limits = [
    [-90, 190],
    [-42, 110],
    [-180, 2],
    [-1, 180], #egentlig 0, men løser mange problemer
    [-102, 102],
    [-90, 240],
    [90, 180] #close, open
]

# Create the kinematics model of the robot
kin_model = Kinematics(geometry, joint_limits)
arduino = ArduinoCommunication()
# Uncomment if you want to use move_to_mouse
camera_input = CameraInput(debug=True, configure=False) # Create a instance of the CameraInput class
#camera_input = CameraInput(debug=False, configure=False) # Create a instance of the CameraInput class
shape_detection = ShapeDetection(debug=False)
# Make the robot trigger the endstops before shutting down
atexit.register(arduino.send_TriggerEndstops)

class RobotArm(object):
    def __init__ (self):
        self.last_point = (None, None)
        self.current_point = (None, None)

    def pick_object_sequence(self):
        """ Picks up an object identified in the picture"""

        # Quit if there no objects in the image and 
        # if the arduino is not ready for a new command
        if (not shape_detection.detected_shapes_coords) or (not arduino.ready):
            return
        # Get the coordinates of the first object in the image 
        xy = camera_input.image_to_coordinates(shape_detection.get_shape_coords(number_shape=0))
        # Create the list of coordinates in 3D-space
        # The robot first moves to center of the object 
        # 3 cm over the ground
        coordinates = [xy[0], xy[1], 30, 0, 3.14, 0, 180]
        print("Control: 3D coordinates ", coordinates)
        # Move the arm to the coordinate, and only
        # proceed if it did so successfully
        if not self.move_to_coordinate(coordinates, 1000):
            return
        # Wait for the arduino to become ready
        # for the next command
        arduino.wait_for_ready(update_camera_feed)
        coordinates[2] = 1
        print("Control: 3D coordinates ", coordinates)
        # Move the arm to the coordinate, and only
        # proceed if it did so successfully
        if not self.move_to_coordinate(coordinates, 1000):
            return
        # Wait for the arduino to become ready
        # for the next command
        arduino.wait_for_ready(update_camera_feed)
        # Close the gripper
        arduino.send_MoveJoint(6, 115, 30)
        # Wait for the arduino to become ready
        # for the next command
        arduino.wait_for_ready(update_camera_feed)
        # Move the arm up
        arduino.send_MoveJoints([0, -20, -30, 0, 0, 0, 115], 1000)
        # Wait for the arduino to become ready
        # for the next command
        arduino.wait_for_ready(update_camera_feed)
        # Open the gripper
        arduino.send_MoveJoint(6, 180, 30)
        
    def move_to_coordinate(self, coordinates, speed):
        """ Moves the endaffector to a given point
        int space. This includes check that the coordinates
        are valid and so on"""

        # Perform IK and check if the angles are valid
        # We remove the last angle (the gripper angle)
        # because this is not relevant for the IK model
        angles_rad = kin_model.doInverseKin(coordinates[:-1])
        # If no angles are returned do not perform the rest of the iteration
        # The destination could be out of reach
        if not angles_rad:
            return False
        # Convert the angles back to degrees (from rad)
        # because that is the unit the arduino operates with
        angles_deg = kin_model.convertRadToDeg(angles_rad)

        # Check if the angles are compatible with the robot
        # arms limitations
        if not kin_model.validateRobotAnglesDeg(angles_deg):
            return False
        # The angles are valid. We add the gripper positon
        # back into the list
        angles_deg.append(coordinates[-1])
        # Send the angles to the robot
        arduino.send_MoveJoints(angles_deg, speed)
        return True

    def move_to_mouse(self, height, speed):
        """ Moves the arm to the point clicked 
        on in the picture by the user""" 
        # Check if there are any points in the list
        if not camera_input.mouse_track_coord:
            return
        # Get the last point in the list
        self.current_point = camera_input.mouse_track_coord[-1]
        # Check wether the point has changed
        # or not to take stress off the arduino
        if self.current_point == self.last_point:
            return
        # Update the previous point
        self.last_point = self.current_point
        # Get the point in robot arm coordinates
        point = camera_input.image_to_coordinates(self.current_point)
        coordinates = [point[0], point[1], height, 0, 3.14, 0, 180]
        self.move_to_coordinate(coordinates, speed)
        arduino.wait_for_ready(update_camera_feed)

def update_camera_feed():
    """ Gets the next frame and perform
    object detection on it """
    camera_input.get_next_frame()
    # Perfrom shape detection
    shape_detection.detect_shapes(camera_input.deskewed_img_resized)


robot_arm = RobotArm()
while True:
    update_camera_feed()
    # Read the serial
    arduino.read_serial()
    #pick_object_sequence()
    robot_arm.move_to_mouse(30, 2000)