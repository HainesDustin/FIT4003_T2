#!/usr/bin/env python

# Simple image listener that looks at ../../../input_images
# Loads images found, and publishes them to /camera/rgb/image_raw 
# for YOLO_v3 to process

# Created: 22/09/19 - Dustin Haines
# Last Modified: 22/09/19 - Dustin Haines

import os
import cv2
import rospkg
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# Constant directory locations
rospack = rospkg.RosPack()
fp = rospack.get_path('yolo_feeders')
IMAGE_DIR = fp + '/../../input_images/'
PROCESSED_DIR = fp + '/../../processed_images/'

# Configuration
VAL_EXTENSIONS = ['.png', '.jpg']
TOPIC = 'camera/rgb/image_raw'
NODE_NAME = 'image_listener'

def findImage():
    # Create a publisher
    pub = rospy.Publisher(TOPIC, Image, queue_size=1)
    # Create an anonymous node to avoid any potential conflict
    rospy.init_node(NODE_NAME, anonymous=True)
    # Set the rate at which the query is run (in Hz)
    # 1 query a second currently, YOLO should process quicker than this
    # but varies based on hardware
    rate = rospy.Rate(1)
    # Create a bridge instance which will convert out image into the right format
    bridge = CvBridge()

    # Launch the listener loop
    while not rospy.is_shutdown():
        # Query the directory
        dir_contents = os.listdir(IMAGE_DIR)
        # Only do processing if there is an image to process
        for item in dir_contents:
            dir_contents.remove(item) if item.split('.')[-1] not in VAL_EXTENSIONS else None
        if len(dir_contents) > 0:
            # Open the first file
            first_file_name = dir_contents[0]
            first_file = cv2.imread((IMAGE_DIR + first_file_name))
            try:
                # Transform into the message
                yolo_image = bridge.cv2_to_imgmsg(first_file)
                # Publish the message
                pub.publish(yolo_image)
                print('Image posted to topic /' + TOPIC)
            except CvBridgeError as err:
                print(err)
            # Move the processed image
            os.rename((IMAGE_DIR + first_file_name), (PROCESSED_DIR + first_file_name))
            print(first_file_name + ' moved to ' + PROCESSED_DIR)
        # Sleep for next iteration
        rate.sleep()

if __name__ == '__main__':
    print 'Image Listener launching with following values'
    print 'IMAGE_DIR: ' + IMAGE_DIR
    print 'PROCESSED_DIR: ' + PROCESSED_DIR
    print 'VAL_EXTENSIONS: ' + str(VAL_EXTENSIONS)
    print 'TOPIC: ' + TOPIC
    print 'NODE_NAME: ' + NODE_NAME
    print '\n'
    try:
        findImage()
    except rospy.ROSInterruptException:
        pass
