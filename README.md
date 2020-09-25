Mobile Robot Localization Module
======

## Documentation
This project is based on ![Husqvarna Research Platform](https://github.com/HusqvarnaResearch/hrp), here you may find the original README file at the bottom. The targeted hardware platform of this project is Huqvarna automower 450x, not so superisingly. 


## Run in simulation
The project is mainly conducted and tested in practice, yet you may get a feeling through the simulation. Additional IMU, GNSS and a monocular camera respect to the physical one are also available in the simulation.  

Necessary packages for the sensors in Gazebo: 
`ros-kinetic-hector-gazebo-pluginss` and `ros-kinetic-geographic-msgs`

To remote control the robot through keyboard in simulation:
```
roslaunch am_gazebo am_gazebo_hrp.launch gui:=true
rosrun am_driver hrp_teleop.py
```

## Run in practice
In general, start with the launch file below. `hrp_teleop` is for keyboard remote control. 
```
roslaunch am_driver_safe automower_hrp.launch
rosrun am_driver hrp_teleop.py
```
Parameter | Value
----------|-------
USB cable control | /dev/ttyACM0
UART control      | /dev/ttyUSB0

### EKF launch
For ekf localization using `robot_localization` package. While the yaml file the node refers to is within `am_description` folder.
```
roslaunch robot_localization ekf_template.launch
```

### IMU&GNSS launch
``` 
roslaunch am_sensors sensors.launch
```
Parameter | Value
----------|-------
... 		| ...
...      	| ...

The CPT filter from (forked)![imu_tool](https://github.com/TianzeLi/imu_tools) is applied. Yet optional low-pass filters for accelerometers and magnetometers are provided in my version. 


### Realsense D435 launch
For Rviz:
```
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud enable_infra1:=false enable_infra2:=false
```
For RTAB-Map:
```
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```
or follow the setup in this project:  
```
roslaunch am_sensors rs_camera.launch
```
To change the camera parameters:
```
rosrun rqt_reconfigure rqt_reconfigure
```

### RTAB-Map launch for D435
``` 
roslaunch rtabmap_ros rtabmap.launch \
    rtabmap_args:="--delete_db_on_start" \
    depth_topic:=/camera/aligned_depth_to_color/image_raw \
    rgb_topic:=/camera/color/image_raw \
    camera_info_topic:=/camera/color/camera_info \
    approx_sync:=false
```
To view the map:
```
rtabmap-databaseViewer ~/.ros/rtabmap.db
```


>## Husqvarna Research Platform
>
>2017 (C) Husqvarna Group
>
>The main repository for Husqvarna Research Platform. This repo contains the basic software and models for you to bring up and run the HRP-enabled mower.
The repository will work with ros (see ros.org) kinetic and indigo releases. Please check documentation for detailed information.
>
>### Usage
>In order to use this, you need to have access to an Automower (with HRP-enabled firmware). If you have a nice research project please do not hesitate to contact us to see if you can 
>get one as well...we are always interested in nice new research!
>
>NOTE! This is not a GENERAL OFFER and it is not a product you can buy! 
>It is not possible to run HRP on commercial available mowers. To get this to work you need to get the mowers from us (loaded with a 
>special firmware not publically available). In order to get a mower, please apply to us with the research idea you have (like a one-pager) 
>and we will evaluate this against all other applicants. Also note that this offer is directed at research institutes and/or univerisities 
>conducting robotics research. For companies, you are free (and most welcome!) to contact us with ideas for possible co-operation 
>where this can be utilized as a part of that co-operation, but the mowers are not generally available to companies.
>
>### Cloning
>Note that this repo contains "large files" and hence you need to have "git-lfs" installed. Please follow the instructions (for example here: https://git-lfs.github.com/) to install!
>
>### Partners
>If you are a HRP partner and want to contribute, please let us know and we will add you to the organization.
>
