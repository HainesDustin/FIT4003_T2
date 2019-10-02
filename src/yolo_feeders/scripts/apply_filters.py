#!/usr/bin/env python

# Listens for images supplied through 'apply_filters' topic
# Applies filter to image based on argument
# Publishes augmented image to camera/rgb/image_raw
# OPTIONS: C (default), DP, HP, D

# Created 01/10/19 - Andrew Bui
# Last Modified 02/10/19 - Andrew Bui 

import sys
import cv2
import numpy as np
import random
import time
import rospkg
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# Constant directory locations
rospack = rospkg.RosPack()
fp = rospack.get_path('yolo_feeders')
NODE_NAME = "filters"
IMAGE_FOLDER = fp + '/scripts/'
OUTPUT_IMAGE_DIR = fp + '/../../output_images/'
INPUT_IMAGE_TOPIC = "apply_filters"
OUTPUT_TOPIC = 'camera/rgb/image_raw'

def images_callback(data):
	# Initial values of filters
	overlay = False
	dead_pixel = False
	hot_pixel = False
	
	# Check what filter argument was provided
	if len(sys.argv) == 2:	
		if sys.argv[1] == "C":
			overlay = True
			# Get overlay image
			foreground = cv2.imread(IMAGE_FOLDER+'cracks-clear-3.png', -1)
		if sys.argv[1] == "HP":
			hot_pixel = True
		if sys.argv[1] == "DP":
			dead_pixel = True
		if sys.argv[1] == "D":
			overlay = True
			# Get overlay image
			foreground = cv2.imread('cracks-clear-3.png', -1)
	# TODO: Determine if there's a default behaviour	
	else: 
		overlay = True
		# Get overlay image
		foreground = cv2.imread(IMAGE_FOLDER+'cracks-clear-3.png', -1)
	
    # Create a CvBridge instance to convert
	bridge = CvBridge()
    # Take the sensor_msgs Image and convert back to cv2
	cv_image = bridge.imgmsg_to_cv2(data)
	filename = str(int(time.time())) + '.png'
	cv2.imwrite(OUTPUT_IMAGE_DIR + filename, cv_image)
	# Read image that was passed through
	suppliedImage = cv2.imread(OUTPUT_IMAGE_DIR + filename)
	print '\n Applying filter to ' + filename
	# Obtain dimensions of supplied image
	background_width, background_height, _ = suppliedImage.shape
	#if background_height > 0 and background_width > 0:
	pixel_defect_x = random.randint(0, background_width)
	pixel_defect_y = random.randint(0, background_height)

	if overlay:
		# Make overlay same size as camera resolution
		foreground = cv2.resize(foreground, (background_height, background_width))
		# Extract the alpha mask of the RGBA image, convert to RGB
		b, g, r, a = cv2.split(foreground)
		overlay_color = cv2.merge((b, g, r))

		# Apply some simple filtering to remove edge noise
		mask = cv2.medianBlur(a, 5)

		h, w, _ = overlay_color.shape
		roi = suppliedImage[0:0 + h, 0:0 + w]

		# Black-out the area behind the logo in our original ROI
		img1_bg = cv2.bitwise_and(roi.copy(), roi.copy(), mask=cv2.bitwise_not(mask))

		# Mask out the logo from the logo image.
		img2_fg = cv2.bitwise_and(overlay_color, overlay_color, mask=mask)

		# Update the original image with our new ROI
		suppliedImage[0:0 + h, 0:0 + w] = cv2.add(img1_bg, img2_fg)
	if dead_pixel:
		# Dead Pixel
		cv2.circle(suppliedImage, (pixel_defect_x, pixel_defect_y), 5, (0, 0, 0), thickness=-1, lineType=8, shift=0)
	if hot_pixel:
		# Hot Pixel
		cv2.circle(suppliedImage, (pixel_defect_x, pixel_defect_y), 5, (45, 67, 237), thickness=-1, lineType=8, shift=0)
		cv2.circle(suppliedImage, (pixel_defect_x, pixel_defect_y), 3, (63, 82, 63), thickness=-1, lineType=8, shift=0)
		cv2.circle(suppliedImage, (pixel_defect_x, pixel_defect_y), 1, (81, 99, 232), thickness=-1, lineType=8, shift=0)

	# Convert cv2 image to imgmsg for YOLO
	yolo_image = bridge.cv2_to_imgmsg(suppliedImage)
	yolo_image.encoding = 'bgr8'
	# Publish to YOLO
	pub.publish(yolo_image)

if __name__ == '__main__':

	pub = rospy.Publisher(OUTPUT_TOPIC, Image, queue_size=1)
	# Initialise the node
	rospy.init_node(NODE_NAME, anonymous=True)
	# Subscribe to topics, specifying the data type and callback function	
	rospy.Subscriber(INPUT_IMAGE_TOPIC, Image, images_callback)
	print '\nListening to topic ' +  INPUT_IMAGE_TOPIC
    # Keeps the listener alive
	rospy.spin()
