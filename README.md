# convert_data

# Require
- Hardware:
    + Ubuntu 18.04 - ROS Melodic
    + Depth camera: Intel Realsense Depth Camera D435i
- Install Intel Realsense SDK:
    + Install: https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
    + Test camera: realsense-viewer
- Install python:
    + $sudo apt install python-pip
    + $pip install pyautogui
- Install ddynamic-reconfigure:
    + $sudo apt-get update
    + $sudo apt-get install ros-melodic-ddynamic-reconfigure
- Install numpy:
    + $sudo apt install ros-melodic-ros-numpy
- Update git submodule:
    + Direct to folder: /convert_data
    + $git submodule init
    + $git submodule update

# RUN
- Check camera is connected: 
    + $ls /dev/video*
    + expect: /dev/video0  /dev/video1  /dev/video2

- Config camera:
    + Open camera: $realsense-viewer
    + On Stereo Module
    + Deselect "Enable Auto Exposure"
    + Set "Controls/Exposure": 20000 - 25000
    + Set "Controls/Gain": 40 - 60

- Run depth camera: $roslaunch realsense2_camera rs_camera.launch

- Run convert_image program: $roslaunch convert_image convert_image.launch

- Run face distance program:
    + https://github.com/dbloisi/unibas_face_distance_calculator
    + $roslaunch convert_image face_distance.launch

- Run detect_obstacle:
    + $roslaunch convert_image detect_obstacle_py.launch