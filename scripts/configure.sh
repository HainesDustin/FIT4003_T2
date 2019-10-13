#/bin/bash

# Script to configure this ROS package to get
# started immediately with using YOLO

# Make the workspace
catkin_make

# Add the workspace to your .bashrc
echo "source ~/FIT4003_T2/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Make the folders required for the image feeder/responder
mkdir input_images
mkdir processed_images
mkdir output_images
mkdir output_bounds

# Download the YOLOv3 weights
cd ~/FIT4003_T2/src/darknet_ros/darknet_ros/yolo_network_config/weights
wget https://pjreddie.com/media/files/yolov3.weights