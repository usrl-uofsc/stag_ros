# STag ROS: A ROS package for the Stable Fiducial Marker System

Currently supporting ROS Kinetic, Melodic and Noetic. Select the branch that matches your ROS distribution.

This package is developed independently from the creators of STag

[Original STag source and marker generation script](https://github.com/bbenligiray/stag)

## Papers
[STag](https://www.sciencedirect.com/science/article/abs/pii/S0262885619300903)

[STag ROS](https://ieeexplore.ieee.org/document/9213977)


## Installation
Follow these steps to install

### Prerequisites
This assumes you already have ROS installed on your device.

### Install STag ROS
```
mkdir -p /path/to/catkin_ws/src
cd /path/to/catkin_ws/src
git clone git@github.com:usrl-uofsc/stag_ros.git
```

### Build STag ROS
```
catkin_make
```

### Build STag ROS with Debug information (alternative)
```
catkin_make -DCMAKE_BUILD_TYPE=Debug
```

## Package configuration
To use the package you need to edit the node config file (**e.g. cfg/single.yaml**) that loads all the parameters and the marker config file (**e.g. cfg/single_config.yaml**).

### STag config file
In the configuration file you can specify information about the STag configuration, the camera topics and the stag_ros node.

### Marker config Yaml file
STag ROS allows the detection of multiple markers and marker bundles as long as they all are from the same HD family. You can specify the individual markers and the marker bundles in a yaml file

## Examples
First source the workspace.
```
source /path/to/catkin_ws/devel/setup.bash (or your shell ex. sh, zsh)
```

Next, download the example bags. If you want to install, be sure to `catkin_make`, then run the following command, then `catkin_make install`. If the script fails, please go into the `scripts/download_bags.sh` file and download the bags manually. Place them into a folder in the root of the stag_ros directory called bags.
```
roslaunch stag_ros download_examples.launch
```

Run any of the following launch commands.
```
roslaunch stag_ros rosNode_single.launch
roslaunch stag_ros rosNode_bundle_2.launch
roslaunch stag_ros rosNode_bundle_4.launch
roslaunch stag_ros rosNodelet_single.launch
roslaunch stag_ros rosNodelet_bundle_2.launch
roslaunch stag_ros rosNodelet_bundle_4.launch
```
