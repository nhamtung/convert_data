# convert_data

# Require
- Hardware:
    + Ubuntu 18.04 - ROS Melodic
    + Depth camera: Intel Realsense Depth Camera D435i
- Install realsense ros:
    + $sudo apt-get install ros-melodic-realsense2-camera

# RUN
- Check camera is connected: 
    + $ls /dev/video*
    + expect: /dev/video0  /dev/video1  /dev/video2

- Run convert_image program:
    + $roslaunch realsense2_camera rs_camera.launch
    + $roslaunch convert_image convert_image.launch

- Run face distance program:
    + https://github.com/dbloisi/unibas_face_distance_calculator
    + $roslaunch realsense2_camera rs_camera.launch
    + $roslaunch convert_image face_distance.launch