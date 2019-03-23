# EKF SLAM
This is a GUI for the extended Kalman filter (EKF) simultaneous localisation and mapping (SLAM), with known associations explained in: ```page 314 of the Probabilistic Robotics book by Sebastian Thrun, Wolfram Burgard and Dieter Fox```.

## Usage
Simply run ```EKF_SLAM_Known_Correspondences.m```:
- Up arrow: vehicle acceleration
- Left/right arrows: counterclockwise and clockwise angular rotation of the vehicle 

## Description
Once a target (landmark) is detected, it is localised over the map on the right hand side. At the same time the vehicle's location is updated using the EKF SLAM framework.
When a target is in the vehicle's FoV, it is coloured as blue.
The targets and vehicle's states at any time is shown the middle column at the top and bottom, respectively.

The uncertainty of the landmarks' locations and vehicle's localisation gradually decreases over time. 
<p align="center">
  <img src="img/myimage.gif" width=1000 >
</p>


The number of landmarks, angular and radial speed, detection error model and covariance matrices can be adjusted as hyper-parameters within the code.

