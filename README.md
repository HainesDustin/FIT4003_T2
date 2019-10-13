# FIT4003 Testing Self Driving Cars - Team 2

## Introduction
This package aims to investigate the impact of visual defects on camera feeds that are fed into AI models used in self driving cars. In particular, the model created by You Only Look Once as implemented by LeggedRobotics [GitHub](https://github.com/leggedrobotics/darknet_ros). Their implementation is included as a submodule for this repository.

## Prerequsities
* Ubuntu 16.04 LTS
* ROS Kinetic - [Install](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* Autoware - [Install](https://gitlab.com/autowarefoundation/autoware.ai/autoware/wikis/Source-Build)
* Nvidia/CUDA drivers - [Install](https://gist.github.com/zhanwenchen/e520767a409325d9961072f666815bb8)

## Getting Started
**It is assumed you have ROS Kinetic installed prior to the use of this**
Clone this repo to your local machine in your home directory
```sh
git clone --recursive git@github.com:HainesDustin/FIT4003_T2.git
```
Switch to the created directory (FIT4003_T2), then run the following
```sh
# Add execution permissions to the configure script
chmod +x ./scripts/originalconfigure.sh
# Run the configure script
./scripts/configure.sh
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

We have a couple of things we need to launch
### Run Autoware
Again, assumed that autoware is installed and ready to use. Navigate to your autoware folder, source setup.bash and launch the runtime manager
```sh
source install/setup.bash
roslaunch runtime_manager runtime_manager.launch
```

### Run Image Transformer
From there, launch a new terminal and run the following to launch the image transformation
```sh
rosrun yolo_feeders apply_filters.py
```

### Run YOLO
Two terminals are required for this, one for each YOLO instance. Alternatively if you are only interested in one feed only launch the one required
```sh
# For the unchanged image feed
roslaunch yolo_feeders yolo_original.launch
# For the transformed image feed
roslaunch yolo_feeders yolo_transform.launch
```

### Run the data collector
```sh
rosrun yolo_feeders box_reader.py
```

## Configuring the scripts
The transformation and collection scripts themselves are located under ```~/FIT4003_T2/src/yolo_feeders/scripts/```
These can be changed on the fly with changes observed once the service is relaunched.

The following have different default and command line parameters that can be used for different configurations. See below

### apply_filters.py
**Defaults**
```sh
NODE_NAME       : filters
IMAGE_FOLDER    : /../../transform_images/
INPUT_TOPIC     : /image_raw
OUTPUT_TOPIC    : /transform/image
IMAGE_CRACK     : cracks.png
IMAGE_NOISE     : noise.png
IMAGE_SCRATCHES : scratches.png
```
**Parameters**
```sh
-f , --filename      Filename of transform image to use, needs to be located in transform_images
-hp, --hotpixel      Apply a random hotpixel
-dp, --deadpixel     Apply a random deadpixel
-c , --cracks        Use the default transform of cracks.png
-n , --noise         Use the default transform of noise.png
-s , --scratches     Use the default transform of scratches.png
-it, --input_topic   Overwrite the default input topic to the one specified
-ot, --output_topic  Overwrite the default output topic to the one specified
```

### box_reader.py
**Parameters**
```sh
-ib, --input_box_topic  : Input topic for bounding box information
-id, --input_det_topic  : Input topic for number of objects detected
-os, --output_file_sum  : Output filename for the summary report
-od, --output_file_det  : Output filename for the detailed report
```

### yolo_original.launch
**Defaults**
```sh
ros_param_file      : $HOME/FIT4003_T2/src/yolo_feeders/config/ros_original.yaml
network_param_file  : $HOME/FIT4003_T2/src/darknet_ros/darknet_ros/config/yolov3.yaml
ros_node            : yolo_original
```

### yolo_transform.launch
**Defaults**
```sh
ros_param_file      : $HOME/FIT4003_T2/src/yolo_feeders/config/ros_transform.yaml
network_param_file  : $HOME/FIT4003_T2/src/darknet_ros/darknet_ros/config/yolov3.yaml
ros_node            : yolo_transform
```


### Questions?
Get in touch with Dusty if you have issues using this or need to commit to the repo