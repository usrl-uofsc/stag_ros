# STag ROS: A ROS package for the Stable Fiducial Marker System

This package is developed independently from the creators of STag.  

[Original STag source and marker generation script](https://github.com/bbenligiray/stag)

[Original publication](https://www.sciencedirect.com/science/article/abs/pii/S0262885619300903)

## Installation
Follow these steps to install

### Prerequisites
This assumes you already have ros installed on your device.
```
sudo apt-get install ros-{DISTRO}-swri-nodelet
```

### Install Git LFS
You may need to install git LFS to download the example bag files. For Ubuntu 18.04 or later:
```
sudo apt-get install git-lfs
```
For earlier versions follow [these instructions](https://packagecloud.io/github/git-lfs/install)

### Install STag ROS
```
mkdir -p /path/to/catkin_ws/src
cd /path/to/catkin_ws/src
```
To download bag files during the clone use this command.
```
git lfs clone git@github.com:usrl-uofsc/stag_ros.git
```
Otherwise use this to download the plain repo without the bag files.
```
git clone git@github.com:usrl-uofsc/stag_ros.git
```
You can always pull the bags later with ```git lfs pull```.

### Build STag ROS
```
catkin_make
```

### Build STag ROS with Debug information (alternative)
```
catkin_make -DCMAKE_BUILD_TYPE=Debug
```

## Package configuration
To use the package you need to edit the node config file (**e.g. cfg/single.yaml**) that loads all the parameters and the marker config file (**e.g. cfg/single.json**).

### Config file
In the configuration file you can specify information about the STag configuration, the camera topic and the marker configuration

### Marker config JSON file
STag ROS allows the detection of multiple markers and marker bundles as long as they all are from the same HD family. You can specify the individual markers and the marker bundles in a JSON file

## Examples
Three examples are available through the bags included with LFS. 

First source the workspace.
```
source /path/to/catkin_ws/devel/setup.bash (or your shell ex. sh, zsh)
```

Then run any of the following launch commands.
```
roslaunch stag_ros rosNode_single.launch
roslaunch stag_ros rosNode_bundle_2.launch
roslaunch stag_ros rosNode_bundle_4.launch
roslaunch stag_ros rosNodelet_single.launch
roslaunch stag_ros rosNodelet_bundle_2.launch
roslaunch stag_ros rosNodelet_bundle_4.launch
```
