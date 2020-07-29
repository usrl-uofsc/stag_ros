# STag ROS: A ROS pacakge for the Stable Fiducial Marker System

[Original STag source and marker generation script](https://github.com/bbenligiray/stag)

## Installation
Follow these steps to install

### Prerequisites
This assumes you already have ros installed on your device.
```
sudo apt-get install ros-{DISTRO}-swri-nodelet
```

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

### Build STag ROS with Debug information
```
catkin_make -DCMAKE_BUILD_TYPE=Debug
```

### Package configuration
#### Config file
In the configuration file you can specify information about the STag configuration, the camera topic and the marker configuration

#### Launch file
We provide both a node and a nodelet implementation with launch files for each one of them

#### Marker config JSON file
STag ROS allows the detection of multiple markers and marker bundles as long as they all are from the same HD family. You can specify the individual markers and the marker bundles in a JSON file