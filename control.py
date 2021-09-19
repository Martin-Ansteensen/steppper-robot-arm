#!/usr/bin/env python3

# Imports
from kinematics import Kinematics
from arduinoCommunication import ArduinoCommunication
from getVideoFeed import CameraInput, ShapeDetection
import imutils, cv2
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
camera_input = CameraInput() # Create a instance of the CameraInput class
shape_detection = ShapeDetection()

# Begin the video stream
vs = cv2.VideoCapture(0)
vs.set(3, camera_input.frame_width)  
vs.set(4, camera_input.frame_height)  
vs.set(5, camera_input.framerate) 
ret, frame = vs.read() # Get the first frame
print("The resolution of the images: ", frame.shape[0], frame.shape[1]) # Print the resolution of the image
# Adjust the resolution in case the camera does not support
# the resolution set in config
camera_input.frame_height = frame.shape[0]  
camera_input.frame_width = frame.shape[1]
past_xy = (None, None)
FIRST_RUN = True
ARDUINO_READY = True
while True:
    ret, frame = vs.read() # Get the next frame.
    frame = imutils.rotate(frame, angle=camera_input.camera_rotation) # Rotate the fram according to the angel provided in the .json file
    camera_input.run(frame)

    if arduino.read_serial():
        ARDUINO_READY = True
    
    if shape_detection.detect(camera_input.deskewed_img_resized):

        current_xy = camera_input.image_to_coordinates(shape_detection.coordinates[0][0], shape_detection.coordinates[0][1])
        if past_xy != current_xy:
            FIRST_RUN = False
            past_xy = current_xy
            coordinates = [past_xy[0], past_xy[1], 30, 0, 3.14, 0, 180]
            print(coordinates)
            angles_rad = kin_model.doInverseKin(coordinates[:-1])

            # If no angles are returned do not perform the rest of the iteration
            if not angles_rad:
                    # Skips the rest of this iteration
                    continue

            angles_deg = kin_model.convertRadToDeg(angles_rad)
            # Only send the values if they are valid
            if kin_model.validateRobotAnglesDeg(angles_deg):
                angles_deg.append(coordinates[-1])  # Add the gipper position
                arduino.send_MoveJoints(angles_deg, 1000)
                
                ARDUINO_READY = False

                while not arduino.read_serial():
                    pass
                coordinates[2] = 1
                print(coordinates)
                angles_rad = kin_model.doInverseKin(coordinates[:-1])

                # If no angles are returned do not perform the rest of the iteration
                if not angles_rad:
                        # Skips the rest of this iteration
                        continue

                angles_deg = kin_model.convertRadToDeg(angles_rad)
                # Only send the values if they are valid
                if kin_model.validateRobotAnglesDeg(angles_deg):
                    angles_deg.append(coordinates[-1])  # Add the gipper position
                    arduino.send_MoveJoints(angles_deg, 500)
                    while not arduino.read_serial():
                        pass
                    arduino.send_MoveJoint(6, 115, 100)

                    while not arduino.read_serial():
                        pass
                    arduino.send_MoveJoints([0, -20, -30, 0, 0, 0, 115], 1000)

                    while not arduino.read_serial():
                        pass
                    arduino.send_MoveJoint(6, 180, 100)


# coordinates = [
#     #x,y,z a,b,c gripper
#     [190, 0,   87, 0,  3.14,  0, 0],
#     [500, -120,   20, 0,  3.14,  0, 0]
# ]

# counter = 0

# FIRST_RUN = True
# # Send and recieve data forever
# while True:
#     # If the arduino is ready to recieve new commands
#     if arduino.read_serial() or FIRST_RUN:
#         FIRST_RUN = False
#         if counter == len(coordinates):
#             #break
#             counter = 0
            
#         # Do the Inverse Kinematics calculations, but 
#         # exclude the gripper open/close position    
#         angles_rad = kin_model.doInverseKin(coordinates[counter][:-1])

#         # If no angles are returned do not perform the rest of the iteration
#         if not angles_rad:
#                 # Skips the rest of this iteration
#                 continue

#         angles_deg = kin_model.convertRadToDeg(angles_rad)
#         # Only send the values if they are valid
#         if kin_model.validateRobotAnglesDeg(angles_deg):
#             angles_deg.append(coordinates[counter][-1])  # Add the gipper position
#             arduino.send_MoveJoints(angles_deg, 1200)

#         counter += 1


