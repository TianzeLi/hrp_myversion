Boundary Guard for Field Robot 
======
**_(Finishing soon but still under construction.)_**
![](doc/media/projectTheme2.png)

## Contents

- [Overview](#Overview)
- [Dependency and installation](#Installation)
- [Launch](#Launch)
- [Documentation](#Documentation)
- [License](#License)
<!-- - [API documentation](#API-documentation) -->
<!-- - [Read more](##Read-more) -->


## Overview
<a name="Overview"></a>

This project is designed for Huqvarna automower 450x as a trial to replace the underlaid wire boundary. The repository is based on ![Husqvarna Research Platform](https://github.com/HusqvarnaResearch/hrp), however has added a localization module and a visual boundary detector, upon additional two IMU, two GNSS and one Intel Realsense D435 depth camera. 

The localization function is considered an active approach that applies Kalman filters to fuse the sensors' outputs and can integrate with the estimated pose from the visual SLAM node. Visual boundary detector then is a more passive method based on image segmentation via explicit image processing pipelines, that is, no heavy learning method is applied, for convience and computational effiency. 

The software is designed under Ubuntu 16.04 and ROS Kinetic. The architecture can be seen below in a conceptual but not strict state machine(not implemented in this project):

![](doc/media/states.svg)


The hardware setup in this project can be abstracted in the diagram below:

![](doc/media/HardwareSetup.svg)




## Dependency and installation
<a name="Installation"></a>

Due to the multiple hectorgeneous sensors we choose, and the customized changes in depent packages, the installation steps is prolonged and rather shown in a seperate [installation page](https://github.com/TianzeLi/hrp_myversion/wiki/Installation) in the repository wiki. 


## Launch
<a name="Launch"></a>

In order for testing, we seperate the functions into different launch files, which can be combined in one overall launch if desired.

### On physical hardware

Launch files   | Functions
-------------- | -------
`roslaunch am_driver_safe automower_hrp.launch`	| Launch the robot
`roslaunch am_sensors sensors.launch`          	| Launch the added sensors
`rosrun am_driver hrp_teleop.py`            	| Control via keyboard
`roslaunch am_driver_safe ekf_template.launch`  | Launch localization
`./am_vision/scripts/boundary_detect.py`        | Run the boundary detect node (may link to ROS later)
`roslaunch am_sensors rtabmap.launch`           | Launch visual-SLAM
`roslaunch am_driver path_follow`				| Run the path follower

Examplary visual boundary detection results:

With Inter Realsense D435 camera:  

![](doc/media/boundary_detect.png)

With iPhone 7 default camera:  

![](doc/media/segResult.png)



### In simulation
Althought not heavily used in this projected, the simulation in Gazebo provides models of the robot and sensors. Two lawn settings are also available in `simulation/am_gazebo/worlds`. In order to launch: 
```
roslaunch am_gazebo am_gazebo_hrp.launch gui:=true
```
The robot in simulation also receives control input via the topic `/cmd/vel`, so `hrp_teleop.py` can be severing here.




## Documentation
<a name="Documentation"></a>

The documentation and miscellanea are available at the project's ![Wiki pages](https://github.com/TianzeLi/hrp_myversion/wiki).


## License
<a name="License"></a>

This repository is under the open source MIT License. 
