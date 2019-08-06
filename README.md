#README

This project contains several plugins to use in Gazebo simulator:

## Requirements

Libraries ```libignition-math4-dev``` and ```libgazebo9-dev``` must be installed before building this package.

Also, package ```gtec_msgs``` must be present in the same work space. This package can be found here:  [https://github.com/valentinbarral/rosmsgs](https://github.com/valentinbarral/rosmsgs)

## Build

After cloning the repository in a catkin workspace:
```
$ catkin_make
```

## UWB Plugin

This plugin models a UWB sensor based on the DW1000 module from Decawave. The plugin simulates the reception of UWB ranging measurements from a set of anchors placed on the scenario. The plugin publish then the next ROS topic: 

* ```/gtec/gazebo/ranging``` Sends ```gtec_msgs::GenericRanging``` messages with the simulated values.


## Magnetic Distortion Plugin

This plugin models a random magnetic interference. Has a subscription to a ROS topic that output magnetic data and modify its values adding a random interference depending on the distance between the vehicle and the interference. The result is published in a new ROS topic with the same name than the original one but ended with */interfered* suffix. 
