# Acquiring Robot US Data

Python scripts for acquiring data with the robot and an attached ultrasound probe. 

## Description

This script can be used for acquiring robot and ultrasound data in a confortable way. It can also be used to get familiar with controlling the robot as well as receiving images/volumes form the philips epiq 7 US station. 


The script does the following:
1. connect with the RobSvr to control the robot
2. connect with the PLUS server to receive US images
3. The current pose of the US probe, which is attached to the end-effector, is determined and then the probe is moved along its z-axis (pointing out of the probe head).


## Getting started

1.) If you want to move in the frame of the probe (as done in the script), make sure the probe is attached correctly / in accordance with the transformation matrix flangeToProbe.
2.) Start RobSvr
3.) Start PLUS server launcher



## Requirements

- PLUS server launcher (PLUS toolkit) with correct config file for the epiq 7
- pyigtl
- numpy
- socket

