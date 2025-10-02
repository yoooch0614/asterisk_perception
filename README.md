# asterisk_perception
`asterisk_perception` metapackage contains the some packages for perception system of ASTERISK.

## Prepare
install the dependencies.
- camera related packages (usb_cam, usb_cam_hardware, libuvc_camera packages)
```
sudo apt install ros-${ROS_DISTRO}-usb-cam
sudo apt install ros-${ROS_DISTRO}-usb-cam-controllers ros-${ROS_DISTRO}-usb-cam-hardware
sudo apt install ros-${ROS_DISTRO}-libuvc-ros
```

- image_reprojection and image_reprojection_plugins package 
```
cd src
git clone https://github.com/yoooch0614/image_reprojection.git
git clone https://github.com/yoooch0614/image_reprojection_plugins.git

```
- aruco_ros packkage
```
git clone https://github.com/pal-robotics/aruco_ros.git
```
- ar_track_alvar packkage
```
git clone https://github.com/ros-perception/ar_track_alvar.git
```

## Install
clone this repository.
```
git clone https://github.com/yoooch0614/asterisk_perception.git
```

and `catkin build` .


## Usage
### asterisk_camera_omnidirectional
this package contains the nodes for omnidirectinal camera system using Insta360 Air.

#### real camera
1. connect the Insta360 Air and Joy controller to PC.
2. roslaunch (test):
    - fixed point-of-view
    ```
    roslaunch asterisk_camera_omnidirectional test_pov_camera_insta360air.launch
    ```
    - controllable point-of-view using joy controller
    ```
    roslaunch asterisk_camera_omnidirectional test_pov_camera_joy_controller_insta360air.launch
    ```

#### simulation
1. connect the Joy controller to PC.
2. roslaunch (prepare):
    ```
    roslaunch asterisk_camera_omnidirectional pov_camera_gazebo.launch
    ```
3. roslaunch (test):
    - fixed point-of-view
    ```
    roslaunch asterisk_camera_omnidirectional test_pov_camera_gazebo.launch
    ```
    - controllable point-of-view using joy controller
    ```
    roslaunch asterisk_camera_omnidirectional test_pov_camera_joy_controller_gazebo.launch
    ```

3. rviz
```
rosrun rviz rviz
```

and load RViz config file at asterisk_camera_omnidirectional/config/pov_camera_insta360_debug.rviz
