# QRPfRA
Quadruped Research Platform for Robotic Applications

## What This Project Includes
- Swift app to acquire depthmap, accelerometer, gyro and magnetometer data and a photo associated with the information. Photos and depthmaps are taken with 5 second intervals and all the sensory information is acquired in that 5 seconds. All these information then load to a Firebase server.
- Python scripts includes preprocessing mainly for depthmaps, converting them to pointclouds with real volumetric information by coordinate system conversion and other helper functions for taking the data from firebase, downloading and processing it.
- Matlab files including the necessary functions for determining the orientation of the phone w.r.t. time and creating a path from sensor outputs to stitch pointcloud information to create 3D map of the environment.
- Simulink files, .stl and .xml files are defining the quadruped's rigidbody simulation in Simscape.
*(3D model of the quadruped is based on to the SpotMicro project)*

With all the included files one can create its own 3D mapping application for automated quadruped system using only FaceID supported device. 
