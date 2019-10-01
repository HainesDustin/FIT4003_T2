# FIT4003 Testing Self Driving Cars - Team 2

## Getting Started
**It is assumed you have ROS Kinetic installed prior to the use of this**
Clone this repo to your local machine in your home directory
```sh
git clone --recursive git@github.com:HainesDustin/FIT4003_T2.git
```
Switch to the created directory (FIT4003_T2), then run the following
```sh
# Add execution permissions to the configure script
chmod +x configure.sh
# Run the configure script
./configure.sh
```
The configure script is simple, and does the following
* Builds a catkin workspace
* Compiles the Darknet, YOLO, and our modules
* Adds the source file to your ~/.bashrc file to load the components in every new terminal
* Loads the source file to your current bash instance
* Downloads the weights required for YOLOv3

## Running YOLO and the scripts
As a precursor to all of this, roscore needs to be running.
In a new terminal window, run
```sh
roscore
```
From there, launch a new terminal and run the following to launch YOLO
```sh
roslaunch darknet_ros yolo_v3.launch
```
YOLO will now sit patiently waiting for an input file

To launch our scripts, they can be run using the following commands
```sh
# For the image feeder
rosrun yolo_feeders imageloader.py
# For the image/bounding box listener
rosrun yolo_feeders imageresponse.py
```

## Configuring the scripts
The scripts themselves are located under ```~/FIT4003_T2/src/yolo_feeders/scripts/```
These can be changed on the fly with changes observed once the service is relaunched

### Questions?
Get in touch with Dusty if you have issues using this or need to commit to the repo