# EKF SLAM
This is a GUI for the extended Kalman filter (EKF) simultaneous localisation and mapping (SLAM), with known associations explained in page 314 of the Probabilistic Robotics book by Sebastian Thrun, Wolfram Burgard and Dieter Fox.

## Usage
Simply run ```EKF_SLAM_Known_Correspondences.m```:
- Up arrow: vehicle acceleration
- Left/right arrows: counter clock-wise and clock-wise angular rotation of the vehicle 
Once a target is detected, it is localised over the map on the right hand side.
When a target is in the vehicle's FoV, it is coloured as blue.
The targets and vehicle's states at any time is shown the middle column at the top and bottom, respectively.

<p align="center">
  <img src="img/myimage.gif" width=800 >
</p>
