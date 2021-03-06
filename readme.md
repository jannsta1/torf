# Background

# Installation
## Dependencies
This package depends on the pyx4 ros package (https://github.com/jannsta1/pyx4). Once you have a catkin workspace with 
pyx4 working in it should be possible to clone the torf package into the same workspace and build using the usual ROS 
build approach (catkin build).  

# Running in simulation
# Quick start
To run in simulation initialise the quick start roslaunch file with: 

```
roslaunch torf cwssim_simulate.launch world:=<WORLD_ON_YOUR_GAZEBO_PATH>
```

note that you will need to provide a gazebo world that is in your GAZEBO_MODEL_PATH, path - custom worlds can be provided on request.

# Batch testing procedure

The *autorun* package is a convenient tool for running batches of experiments. It uses the roslaunch api to  

```
cd autorun_dirtorf/src/autorun
python 
```


# Running on hardware
## Hardware dependencies
This software was designed to operate with a bluefox camera but could be readily adopted to any other camera system with 
a ROS driver. The package should operate with any multirotor that uses a PX4 as its main flight controller.

# Contributing
## unit tests 
Unit tests are stored in ```src/test``` folder - please run these before submitting a pull request. 