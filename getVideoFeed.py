import cv2, imutils, json, os, os.path
import numpy as np


class CameraInput(object):
	"""  The user chooses four points on the screen. The image 
	inside these four points is then deskewed  """
	
	def __init__(self, debug, configure):
		"""  Declare variables and load settings from
		json file  """
		self.debug = debug # Only show the deskewed image
		self.configure = configure
		self.x_real, self.y_real = None, None
		self.original_frame = None # The original fram in full resolution
		self.draw_deskew_img = None  # The image the user draws the four points for deskewing on (original resized)
		self.deskewed_img = None # The deskewed image in full resolution
		self.deskewed_img_resized = None  # The deskewed image in a display friendly resolution
		self.current_frame = None

		if self.configure:
			# The window where the user defines the area for deskewing
			self.drawing_window_deskew = "Define deskewing area"  
			cv2.namedWindow(winname= self.drawing_window_deskew, flags=cv2.WINDOW_NORMAL)
			cv2.setMouseCallback(self.drawing_window_deskew, self.collect_user_deskew)
		if self.debug:
			# The window where the deskewed picture is displayed
			self.window_deskew = "Deskewed"  
			cv2.namedWindow(winname= self.window_deskew, flags=cv2.WINDOW_NORMAL)
			cv2.setMouseCallback(self.window_deskew, self.deskew_mouse_track)
			self.mouse_track_coord = []

		# Path to the config file
		self.config_path = "config.json"

		# Open config file containing settings
		with open(self.config_path) as json_file:
			data = json.load(json_file)
			json_file.close()

		# Camera settings
		camera = data["camera"]
		self.frame_height = camera["video_stream_height"]
		self.frame_width = camera["video_stream_width"]
		self.framerate = camera["framerate"]
		self.camera_rotation = camera["video_rotation"]


		# The height of the users drawing windows
		# setting this low (200) will make the program
		# lag less, and will not affect the image processing,
		# except that the images the users sees on this screen
		# will appear shitty
		self.user_window_height = data["user_resolution"]


		# Dict holding the coordinates for both deskewing
		# These points are from the resized images
		self.user_drawing_img_coords = {}
		self.user_drawing_img_coords["deskew"] = [[] for i in range(4)] 
		
		# Copy of self.user_drawing_img_coords["deskew"] except that they are 
		# scaled to fit the original image and not the resized one
		self.org_img_coords = {}
		self.org_img_coords["deskew"] = [None for i in range(4)]  
		
		# Check that the points provided in the .json file are valid
		if len(data["deskewing_coordinates"]) == 4:
			print("You have these four points as default for deskewing:")
			for (i, point) in enumerate(data["deskewing_coordinates"]):
				print(point)
				conditions = [
						point[0] >= 0,  # Check that x and y is not negative
						point[1] >= 0,
						# Check that x and y is inside the image
						# Since these are the drawing coordinates we have to scale 
						# Them to fit the image
						point[0]*self.frame_height/self.user_window_height <= self.frame_width,  
						point[1]*self.frame_height/self.user_window_height <= self.frame_height 
				]
				# If the point is invalid
				if not all(conditions): 
					print("One of the points are invalid. Please edit the points in self.config_path")
					self.user_drawing_img_coords["deskew"] = [(0,0), (0,0), (0,0), (0,0)]
					break

				# The points are valid and added to the list of coordinates
				self.user_drawing_img_coords["deskew"][i] = (point[0], point[1])  
		else:
			print("There are either too many or too few points in deskew.json")

		# Begin the video stream
		self.videostream = cv2.VideoCapture(0)
		self.videostream.set(3, self.frame_width)  
		self.videostream.set(4, self.frame_height)  
		self.videostream.set(5, self.framerate) 
		ret, frame = self.videostream.read() # Get the first frame
		print("The resolution of the images: ", frame.shape[0], frame.shape[1]) # Print the resolution of the image
		# Adjust the resolution in case the camera does not support
		# the resolution set in config
		self.frame_height = frame.shape[0]  
		self.frame_width = frame.shape[1]


	def collect_user_deskew(self, event, x, y, flags, param):
		"""  Is called at mouse activity in the 'Draw deskewing area'
		and collects the user input for the area that is going to be
		deskewed  """

		if event == cv2.EVENT_LBUTTONUP:
			# Reset the drawing process if there already are four points in the list
			if len(self.user_drawing_img_coords["deskew"]) == 4: 
				self.user_drawing_img_coords["deskew"] = []				
			# The list is not full anymore (or never was) and the point is added to the list
			self.user_drawing_img_coords["deskew"].append((x,y))
			# If the list is complete with four points,
			# save them to the config file
			if len(self.user_drawing_img_coords["deskew"]) == 4: 
				with open(self.config_path, 'r') as infile:
					data = json.load(infile)
					data["deskewing_coordinates"] = self.user_drawing_img_coords["deskew"]
				with open(self.config_path, 'w') as outfile:
					json.dump(data, outfile, indent=4)
					print("Saved current coordinates for deskewing to config")

	def map(self, x, in_min, in_max, out_min, out_max):
		""" Map function """
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

	def image_to_coordinates(self, coord):
		""" Takes a the coordinates of a pixel in 
		the deskewed image and maps it to the coordinate
		in the robot-coordinate-system that matches the point
		in the picture"""

		# i x fra bredden av deskewed til 0 (0 er til høyre)
		# i x til 9 til 26,3
		x_real = self.map(coord[0], self.deskewed_shape[0], 0, 90, 263)

		# i y fra høyden av bildet (bunn) til 0
		# i y til 11 til -11
		y_real = self.map(coord[1], self.deskewed_shape[1], 0, 110, -110)
	
		return (x_real, y_real)

	def deskew_mouse_track(self, event, x, y, flags, param):
		""" When the user cliks on the deskewed image
		the pixel it clicks is printed to the console 
		together with the robot-coordinate-system coordinate"""
		if event == cv2.EVENT_LBUTTONUP:
			self.mouse_track_coord.append((x,y))
			self.x_real, self.y_real = self.image_to_coordinates((x, y))

			print("The image coordinates " + str(x) + ", " + str(y) + " matches these real life coordniates " + str(self.x_real) + ", " + str(self.y_real))

	def draw_mouse_track(self):
		""" Draws all the points the user has 
		cliked on in the deskewed image"""
		for i in self.mouse_track_coord:
			cv2.circle(self.deskewed_img_resized, i, 3, (255, 0, 0), -1)

	def draw_deskew(self):
		"""  Draws the user input onto the 'Draw deskewing area'. This 
		shows the user what part of the image will be deskewed  """
		for i in range(len(self.user_drawing_img_coords["deskew"])):
			if i == 0: 
				# Draw a dot representing the first point
				cv2.circle(self.draw_deskew_img, self.user_drawing_img_coords["deskew"][0], 5, (255,0,0), -1)
			else: 
				# Draw a line from the point before and the current point
				cv2.line(self.draw_deskew_img, self.user_drawing_img_coords["deskew"][i-1], self.user_drawing_img_coords["deskew"][i],(255,0,0),5)
			if i == 3:
				# Draw a line between the last and first point
				cv2.line(self.draw_deskew_img, self.user_drawing_img_coords["deskew"][-1], self.user_drawing_img_coords["deskew"][0],(255,0,0),5)
	
	def order_points(self, points):
		"""  Order the points in the following order for the perspective transfromation
		to work: top left, top right, bottom right, bottom left  """
		sorted_points = np.zeros((4, 2), dtype = "float32")
		sum_points = points.sum(axis = 1)
		sorted_points[0] = points[np.argmin(sum_points)]  # The top-left point will have the smallest sum
		sorted_points[2] = points[np.argmax(sum_points)]  # The bottom-right point will have the largest sum
		diff = np.diff(points, axis = 1)  # Compute the difference between the points
		sorted_points[1] = points[np.argmin(diff)]  # The top-right point will have the smallest difference
		sorted_points[3] = points[np.argmax(diff)]  # The bottom-left will have the largest difference
		return sorted_points

	def four_point_transform(self, points, img):
		"""  Deskews the image """
		rect = self.order_points(points)  # Sort the points so that they form a rectangle
		(tl, tr, br, bl) = rect # Unpack the points (assign each point to its own variable)
		# Compute the width of the new image, which will be the
		# maximum distance between bottom-right and bottom-left
		# x-coordiates or the top-right and top-left x-coordinates
		widthA = np.sqrt(((br[0] - bl[0]) ** 2) + ((br[1] - bl[1]) ** 2))
		widthB = np.sqrt(((tr[0] - tl[0]) ** 2) + ((tr[1] - tl[1]) ** 2))
		maxWidth = max(int(widthA), int(widthB))
		# Compute the height of the new image, which will be the
		# maximum distance between the top-right and bottom-right
		# y-coordinates or the top-left and bottom-left y-coordinates
		heightA = np.sqrt(((tr[0] - br[0]) ** 2) + ((tr[1] - br[1]) ** 2))
		heightB = np.sqrt(((tl[0] - bl[0]) ** 2) + ((tl[1] - bl[1]) ** 2))
		maxHeight = max(int(heightA), int(heightB))
		# Construct the set of destination points to obtain a "birds eye view",
		# (i.e. top-down view) of the image, again specifying points
		# in the top-left, top-right, bottom-right, and bottom-left
		# order
		dst = np.array([
			[0, 0],
			[maxWidth - 1, 0],
			[maxWidth - 1, maxHeight - 1],
			[0, maxHeight - 1]], dtype = "float32")
		# Compute the perspective transform matrix and then apply it
		M = cv2.getPerspectiveTransform(rect, dst)
		deskewed = cv2.warpPerspective(img.copy(), M, (maxWidth, maxHeight))
		return deskewed
		
	def process(self, frame):
		"""  Update the drawing image to the newest frame and deskews
		the image if the user has drawn four points  """

		# Rotate the fram according to the angel provided in the .json file
		frame = imutils.rotate(frame, angle=self.camera_rotation) 
		self.original_frame = frame
		self.draw_deskew_img = imutils.resize(frame, height = self.user_window_height) # Resize the frame to fit it on the user's screen
		if self.configure:
			self.draw_deskew() # Draw the user input related to deskewing onto the image
			cv2.imshow(self.drawing_window_deskew, self.draw_deskew_img)

		if len(self.user_drawing_img_coords["deskew"]) == 4:

			# The coordinates collected are from the drawing image 
			# (which is another size than the original  image) 
			# to get the best resolution we deskew the original image, 
			# (and not the resized one). Therefore we need to adjust the coordinates
			deskew_scale = self.frame_height/self.user_window_height
			for (i, e) in enumerate(self.user_drawing_img_coords["deskew"]):
				self.org_img_coords["deskew"][i] = (e[0]*deskew_scale, e[1]*deskew_scale)
			
			deskew_points = np.array((self.org_img_coords["deskew"]), dtype = "float32")  # Convert the list to a numpy array
			self.deskewed_img = self.four_point_transform(deskew_points, self.original_frame)  # Deskew the image
			self.deskewed_img_resized = imutils.resize(self.deskewed_img, height = self.user_window_height)
			if self.debug:
				self.draw_mouse_track()
				cv2.imshow(self.window_deskew, self.deskewed_img_resized)
			# Inntil videre operer vi med det resiza bilde av deskew
			# men dette burde endres ved hjelp av skalering slik vi gjør
			# med deskew scale

			# Bredde og høyde
			self.deskewed_shape = (self.deskewed_img_resized.shape[1], self.deskewed_img_resized.shape[0])

	def saveImage(self):
		"""  Saves the current frame to the images folder """
		DIR = "/home/pi"
		numFiles = len([name for name in os.listdir(DIR) if os.path.isfile(os.path.join(DIR, name))])
		cv2.imwrite(DIR+"/image"+str(numFiles+1)+ ".png", self.deskewed_img)
		print("Saved deskewed image")

	def get_next_frame(self):
		""" Gets the next frame, proccesses (deskewing) it
		and checks if the user wants to quit or
		save the frame """
		ret, frame = self.videostream.read() # Get the next frame.
		self.process(frame) # Process the user input data

		# Get the status of the keyboard keys
		key = cv2.waitKey(1) & 0xFF 
		# Exit the program if the user presses 'x'
		if key == ord("q"): 
			# Cleanup before exit.
			cv2.destroyAllWindows()
			exit()
		
		# Save the image if the user presses 's'
		if key == ord("s"):
			self.saveImage()

class ShapeDetection(object):

	def __init__(self, debug) -> None:
		self.debug = debug
		self.org_img = None
		self.org_img_dimensions = None
		self.detected_shapes_coords = []

		self.shapes_detected_window = "Shapes detected"  
		cv2.namedWindow(winname= self.shapes_detected_window, flags=cv2.WINDOW_NORMAL)
		if self.debug:
			self.processd_img_window = "Processed image"  
			cv2.namedWindow(winname= self.processd_img_window, flags=cv2.WINDOW_NORMAL)

	def process_img(self, img):
		""" Prepares the image for shape
		detection """

		# Convert image into grayscale image
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		# Set threshold of gray image
		_, threshold = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        # Remove noise from the image
		kernel = np.ones((5,5), np.uint8)
		dilate = cv2.dilate(threshold, kernel, iterations=2)
		erode = cv2.erode(dilate, kernel, iterations=3)
		if self.debug:		
			# Display the processed image to the user
			cv2.imshow(self.processd_img_window, erode)
		
		return erode

	def detect_shapes(self, img):
		""" Finds shapes in the given image """
		self.org_img = img
		self.org_img_dimensions = (self.org_img.shape[0], self.org_img.shape[1])

		processed_img = self.process_img(self.org_img)
		self.detected_shapes_coords = []
		contours, _ = cv2.findContours(processed_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		org_img_area = self.org_img_dimensions[0]*self.org_img_dimensions[1]
		# Ignore the first counter because
		# cv2.findContours detects the whole image as a shape
		for contour in contours[1:]:

			# Approximate the shape
			approx = cv2.approxPolyDP(
				contour, 0.01 * cv2.arcLength(contour, True), True)
			
			(x, y, w, h) = cv2.boundingRect(approx)

			# The shape can not be to big or small or 
			# the robot will not be able to pick it up
			if not org_img_area*0.02 < w*h < org_img_area*0.15:
				continue
			
			# Draw the contours onto the picture
			cv2.drawContours(self.org_img, [contour], 0, (0, 0, 255), 5)
			
			# Fin the center point of shape
			M = cv2.moments(contour)
			if M['m00'] != 0.0:
				x = int(M['m10']/M['m00'])
				y = int(M['m01']/M['m00'])

			self.detected_shapes_coords.append((x,y))

		# Display the image with the contours
		cv2.imshow(self.shapes_detected_window, self.org_img)

		if self.detected_shapes_coords:
			return True
		else:
			return False
	
	def get_shape_orientation(self):
		pass

	def get_shape_coords(self, number_shape):
		if self.detected_shapes_coords:
			return self.detected_shapes_coords[number_shape]

if __name__ == "__main__":

	camera_input = CameraInput(configure=True, debug=True) # Create a instance of the CameraInput class
	shape_detection = ShapeDetection(debug=False)

	while True:
		camera_input.get_next_frame()
		if shape_detection.detect_shapes(camera_input.deskewed_img_resized):
			print(camera_input.image_to_coordinates(shape_detection.get_shape_coords(0)))