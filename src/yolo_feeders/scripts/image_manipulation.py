#!/usr/bin/env python

import cv2
import numpy as np
import random

if __name__ == '__main__':
	overlay = False
	dead_pixel = False
	hot_pixel = False
	print("Select a defect to replicate:\nC. Crack\nHP. Hot Pixel\nDP. Dead Pixel\nD. Dirt")
	choice = input()

	if choice == "C":
		overlay = True
		# Get overlay image
		foreground = cv2.imread('cracks-clear-3.png', -1)
	if choice == "HP":
		hot_pixel = True
	if choice == "DP":
		dead_pixel = True
	if choice == "D":
		overlay = True
		# Get overlay image
		foreground = cv2.imread('cracks-clear-3.png', -1)

	# Open the camera
	cap = cv2.VideoCapture(0)

	# Set initial value of weights
	ret, background = cap.read()
	background = cv2.flip(background, 1)
	background_width, background_height, _ = background.shape

	pixel_defect_x = random.randint(0, background_width)
	pixel_defect_y = random.randint(0, background_height)
	foreground = cv2.imread('cracks-clear-3.png', -1)
	# Make overlay same size as camera resolution
	foreground = cv2.resize(foreground, (background_height, background_width))

	while True:
		# Read the background
		ret, background = cap.read()
		background = cv2.flip(background, 1)

		if overlay:
			# Extract the alpha mask of the RGBA image, convert to RGB
			b, g, r, a = cv2.split(foreground)
			overlay_color = cv2.merge((b, g, r))

			# Apply some simple filtering to remove edge noise
			mask = cv2.medianBlur(a, 5)

			h, w, _ = overlay_color.shape
			roi = background[0:0 + h, 0:0 + w]

			# Black-out the area behind the logo in our original ROI
			img1_bg = cv2.bitwise_and(roi.copy(), roi.copy(), mask=cv2.bitwise_not(mask))

			# Mask out the logo from the logo image.
			img2_fg = cv2.bitwise_and(overlay_color, overlay_color, mask=mask)

			# Update the original image with our new ROI
			background[0:0 + h, 0:0 + w] = cv2.add(img1_bg, img2_fg)
		if dead_pixel:
			# Dead Pixel
			cv2.circle(background, (pixel_defect_x, pixel_defect_y), 5, (0, 0, 0), thickness=-1, lineType=8, shift=0)
		if hot_pixel:
			# Hot Pixel
			cv2.circle(background, (pixel_defect_x, pixel_defect_y), 5, (45, 67, 237), thickness=-1, lineType=8, shift=0)
			cv2.circle(background, (pixel_defect_x, pixel_defect_y), 3, (63, 82, 63), thickness=-1, lineType=8, shift=0)
			cv2.circle(background, (pixel_defect_x, pixel_defect_y), 1, (81, 99, 232), thickness=-1, lineType=8, shift=0)

		cv2.imshow('a', background)
		k = cv2.waitKey(10)

		# Press q to break
		if k == ord('q'):
			break

	# Release the camera and destroy all windows
	cap.release()
	cv2.destroyAllWindows()
