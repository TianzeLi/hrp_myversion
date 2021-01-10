Boundary Guard for Field Robot 
======


## Contents

- [Overview](##Overview)
- [Installation](##Installation)
- [Launch](##Launch)
- [Documentation](##Documentation)
- [License](##License)
<!-- - [API documentation](#API-documentation) -->
<!-- - [Read more](##Read-more) -->


## Overview
This project is designed for Huqvarna automower 450x as a trial to replace the underlaid wire boundary. The repository is based on ![Husqvarna Research Platform](https://github.com/HusqvarnaResearch/hrp), however has added a localization module and a visual boundary detector, upon additional two IMU, two GNSS and one Intel Realsense D435 depth camera. 

The localization function is considered an active approach to limit the robot within its operation area. It applies Kalman filters to fuse the sensors' outputs and can integrate with the estimated pose from the visual SLAM node. Visual boundary detector to the contrary is considered to be a more passive method to bound the area. It is based on image segmentation via explicit image processing pipelines, that is, no heavy learning method is applied, for convience and computational effiency. 

The hardware setup in this project can be abstracted in the diagram below:
![]{doc/media/HardwareSetup.svg } 

## Installation
Necessary packages for the sensors in Gazebo: 
`ros-kinetic-hector-gazebo-pluginss` and `ros-kinetic-geographic-msgs`


## Launch

### On physical hardware 

### In simulation
Althought not heavily used in this projected, the simulation in Gazebo provides models of the robot and sensors. Two lawn settings are also available in `simulation/am_gazebo/worlds`. In order to launch: 
```
roslaunch am_gazebo am_gazebo_hrp.launch gui:=true
```
The robot in simulation also receives control input via the topic `/cmd/vel`, so `hrp_teleop.py` can be severing here.


## Documentation
The documentation and miscellanea are available at the project's ![Wiki pages](https://github.com/TianzeLi/hrp_myversion/wiki).


## License
This repository is under the open source MIT License. 
