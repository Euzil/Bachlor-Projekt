# -*- coding: utf-8 -*-
"""
Created on Tue Mar  8 11:16:57 2022

@author: Jonas

Script for:

- US data acquisiton for 3D Volumes
- Movement of the robot
- XL143 Probe attached accordingly

For getting the US images from the philips epiq 7 the PLUS toolkit is needed 
with an according configuration file for the us station.



"""
import socket
import struct
import sys
from PIL import Image
import numpy as np
import math

from datetime import datetime

import pyigtl  # pylint: disable=import-error
import time

import helperRobot as hR

def movex(rangex):     
    new = hR.getWorldToTool(s, BUFFER_SIZE, flangeToProbeXL143)
    new[0,3] = new[0,3] + rangex

     
    worldToFlangeCommand = np.matmul(new, hR.homInv(flangeToProbeXL143))

    # move robot according to mode
    if robotMode == 'SmartServo':
        
        poseStringForRobot = hR.make_cmd("MovePTPHomRowWiseMinChangeRT", worldToFlangeCommand)
        
    else:
        
        poseStringForRobot = hR.make_cmd("MovePTPHomRowWiseMinChange", worldToFlangeCommand)
            
    # command pose to robot
    hR.command(s, BUFFER_SIZE, poseStringForRobot)

    # wait until position is reached (can be adjusted...)
    hR.delay(delayBetweenPoses)

    ## save volumes and robot data

    timestamp = hR.millis()

    worldToToolMeas = hR.getWorldToTool(s, BUFFER_SIZE, flangeToProbeXL143)
    print('worldToToolMeas:  ', worldToToolMeas)

    jointPos = hR.getJointPos(s, BUFFER_SIZE)
    print('jointPos:  ', jointPos)

    jointTorques = hR.getJointTorques(s, BUFFER_SIZE)
    print('jointTorques:  ', jointTorques)

    jointExtTorques = hR.getJointTorques(s, BUFFER_SIZE)
    print('jointExtTorques:  ', jointTorques)

    forceTorqueEE = hR.getForceTorque(s, BUFFER_SIZE)
    print('forceTorqueEE', forceTorqueEE)
    
def movey(rangey):     
    
    new = hR.getWorldToTool(s, BUFFER_SIZE, flangeToProbeXL143)
    new[1,3] = new[1,3] + rangey

     
    worldToFlangeCommand = np.matmul(new, hR.homInv(flangeToProbeXL143))

    # move robot according to mode
    if robotMode == 'SmartServo':
        
        poseStringForRobot = hR.make_cmd("MovePTPHomRowWiseMinChangeRT", worldToFlangeCommand)
        
    else:
        
        poseStringForRobot = hR.make_cmd("MovePTPHomRowWiseMinChange", worldToFlangeCommand)
            
    # command pose to robot
    hR.command(s, BUFFER_SIZE, poseStringForRobot)

    # wait until position is reached (can be adjusted...)
    hR.delay(delayBetweenPoses)

    ## save volumes and robot data

    timestamp = hR.millis()

    worldToToolMeas = hR.getWorldToTool(s, BUFFER_SIZE, flangeToProbeXL143)
    print('worldToToolMeas:  ', worldToToolMeas)

    jointPos = hR.getJointPos(s, BUFFER_SIZE)
    print('jointPos:  ', jointPos)

    jointTorques = hR.getJointTorques(s, BUFFER_SIZE)
    print('jointTorques:  ', jointTorques)

    jointExtTorques = hR.getJointTorques(s, BUFFER_SIZE)
    print('jointExtTorques:  ', jointTorques)

    forceTorqueEE = hR.getForceTorque(s, BUFFER_SIZE)
    print('forceTorqueEE', forceTorqueEE)

def movez(rangez):     
    new = hR.getWorldToTool(s, BUFFER_SIZE, flangeToProbeXL143)
    new[2,3] = new[2,3] + rangez

     
    worldToFlangeCommand = np.matmul(new, hR.homInv(flangeToProbeXL143))

    # move robot according to mode
    if robotMode == 'SmartServo':
        
        poseStringForRobot = hR.make_cmd("MovePTPHomRowWiseMinChangeRT", worldToFlangeCommand)
        
    else:
        
        poseStringForRobot = hR.make_cmd("MovePTPHomRowWiseMinChange", worldToFlangeCommand)
            
    # command pose to robot
    hR.command(s, BUFFER_SIZE, poseStringForRobot)

    # wait until position is reached (can be adjusted...)
    hR.delay(delayBetweenPoses)

    ## save volumes and robot data

    timestamp = hR.millis()

    worldToToolMeas = hR.getWorldToTool(s, BUFFER_SIZE, flangeToProbeXL143)
    print('worldToToolMeas:  ', worldToToolMeas)

    jointPos = hR.getJointPos(s, BUFFER_SIZE)
    print('jointPos:  ', jointPos)

    jointTorques = hR.getJointTorques(s, BUFFER_SIZE)
    print('jointTorques:  ', jointTorques)

    jointExtTorques = hR.getJointTorques(s, BUFFER_SIZE)
    print('jointExtTorques:  ', jointTorques)

    forceTorqueEE = hR.getForceTorque(s, BUFFER_SIZE)
    print('forceTorqueEE', forceTorqueEE)
    
def initialize():
    
    desired_position = np.array ([[ 0.627 , 0.-4.71 ,  0.619 ,  1053.44 ],[-0.46 , -0.86, -0.18 , -28.59],[ 0.62 , -0.17, -0.76, -197.43],[ 0 ,  0,  0 ,  1.0]])
    #desired_position = np.array ([[-0.33, -0.77,  0.54,  1141.65],[-0.53, -0.316, -0.79, -81],  [ 0.77, -0.55, -0.3, -290.77], [0 ,0 , 0, 1]])


         
         # move robot according to mode
    if robotMode == 'SmartServo':
             
       poseStringForRobot = hR.make_cmd("MovePTPHomRowWiseMinChangeRT", desired_position)
             
    else:
             
       poseStringForRobot = hR.make_cmd("MovePTPHomRowWiseMinChange", desired_position)
                 
         # command pose to robot
    hR.command(s, BUFFER_SIZE, poseStringForRobot)
         
         # wait until position is reached (can be adjusted...)
    hR.delay(delayBetweenPoses)

         ## save volumes and robot data
         
    timestamp = hR.millis()
         
    worldToToolMeas = hR.getWorldToTool(s, BUFFER_SIZE, flangeToProbeXL143)
    print('worldToToolMeas:  ', worldToToolMeas)
         
    jointPos = hR.getJointPos(s, BUFFER_SIZE)
    print('jointPos:  ', jointPos)
         
    jointTorques = hR.getJointTorques(s, BUFFER_SIZE)
    print('jointTorques:  ', jointTorques)
         
    jointExtTorques = hR.getJointTorques(s, BUFFER_SIZE)
    print('jointExtTorques:  ', jointTorques)
         
    forceTorqueEE = hR.getForceTorque(s, BUFFER_SIZE)
    print('forceTorqueEE', forceTorqueEE)
         

    print('Start position reached')
    
    
    
    
## set up port for robot connection
TCP_IP = '127.0.0.1'
TCP_PORT = 8000
BUFFER_SIZE = 1024

## transformation of mounted ultrasound probe, here the XL143
##### WARNING: has to be adapted if another probe is used! Make sure that 
##### the probe holder is monted in accordance with the transformation matrix.

flangeToProbeXL143 = np.array([[0.7071, 0, 0.7071, 30],[0,1,0,0],[-0.7071,0,0.7071,135],[0,0,0,1]])

## connect and initialize robot
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))
s.recv(BUFFER_SIZE)

robotMode = 'DirectServo' # DirectServo (normal mode) or SmartServo 

# initialize robot parameters
# go to helperRobot file to adapt initialization parameters (velocities, damping, ...)
hR.initRobot(s, BUFFER_SIZE, robotMode)

## open IGTLinkClient, PLUS server sends images
client = pyigtl.OpenIGTLinkClient(host="127.0.0.1", port=18944)

# get initial image from US station to define shape
message = client.wait_for_message("Image_Transducer", timeout=3)
#volShape = np.shape(message.image)


## define data struct to save measurements
tp = np.dtype([('Timestamp', 'f8'), ('ProbePoseCommand', 'f8', (4, 4)),
               ('ProbePoseMeas', 'f8', (4, 4)), ('JointPos','f8', (1,7)), 
               ('JointTorques', 'f8', (1,7)), ('JointExtTorques', 'f8', (1,7)),
               ('ForceTorqueEE', 'f8', (1,6))])


# counter to fill measurements data array
countMeas = 0

# total number of measurements, which should be saved. 
# TODO: This should be improved...
totNumbMeas = 10

# save data in struct array
dataExp = np.zeros(totNumbMeas, dtype=tp)

## name for experiment
expName = 'TransProbeMovement'
saveData = False


## get current pose of the probe as startpose for the movement
worldToToolStart = hR.getWorldToTool(s, BUFFER_SIZE, flangeToProbeXL143)

print('startPoseTool: ', worldToToolStart)

# wait 5 s
hR.delay(5000)

# might be useful in SmartServo mode to ensure that every pose is reached before
# saving the robot stat and the us images.
delayBetweenPoses = 1500 # in ms

#def movementalong x

# =============================================================================
# ###Move in loops
# x = int(input("movement in x direction"))
# y = int(input("movement in y direction"))
# z = int(input("movement in z direction"))
# loops = int(input("number of loops"))
#      
# initialize()   
# 
# movez(-z)
# for i in range(1,loops+1): # loop to move the robot in loops
# 
#     movement = (-1)**i
#     
#     movex( -movement * x)
#     movey( y)
#     
# initialize()
# =============================================================================

#move in rectangle
# =============================================================================
# 
# initialize()  
# 
# x = int(input("movement in x direction"))
# y = int(input("movement in y direction"))
# z = int(input("movement in z direction"))
# loops = int(input("number of loops"))
#      
#  
# 
# movez(-z)
# movex( x)
# movey( y)
# movex( -x)
# y = y - 10
# movey( -y)
# 
# for i in range(1,loops): # loop to move the robot in loops
# 
#     movement = (-1)**i
#     
#     x = x - 10
#     movex( -movement * x)    
# 
#     y = y - 10
#     movey( -movement * y)
#     
#     
# initialize()
# =============================================================================


# =============================================================================
# ###Move in L loops
# 
# x = int(input("movement in x direction"))
# y = int(input("movement in y direction"))
# z = int(input("movement in z direction"))
# loops = int(input("number of loops"))
#      
# initialize()   
# 
# for i in range(1,loops+1):
#     movez(-z)
#     movex(x)
#     movex(-x)
#     movez(z)
#     movey(y)
#     
# initialize()
# =============================================================================

# =============================================================================
# ###Move in L loops
# 
# x = int(input("movement in x direction"))
# y = int(input("movement in y direction"))
# z = int(input("movement in z direction"))
# loops = int(input("number of loops"))
#      
# initialize()   
# 
# for i in range(1,loops+1):
#     movez(-z)
#     movex(x)
#     movex(-x)
#     movez(z)
#     movey(y)
#     
# initialize()
# =============================================================================










############################## WARNING ########################################
## script moves robot with us probe into +z direction for 20 mm (along the probe axis)
## outgoing from its starting position

## please make sure that the probe is accordingly mounted so that the 
## transformation matrix 'flangeToProbeXL143' is correct. 
###############################################################################
# =============================================================================
# new = hR.getWorldToTool(s, BUFFER_SIZE, flangeToProbeXL143)
# 
# for o in range(10):
#      
#      
#      # go in z-direction of us probe
#      #toolToNewTarget = np.identity(4)
# 
#      # Move Probe 20 mm into z-Direction of the probe
#      #toolToNewTarget[0:3,0:3] = hR.rotxHMD(o+30)[0:3,0:3]
#      #toolToNewTarget[1,3] = o*20
#      
#      #desired_position = np.array ([[ 0.627 , -0.471 ,  0.619 ,  1053.44 ],[-0.46 , -0.86, -0.18 , -28.59],[ 0.62 , -0.17, -0.76, -197.43+(-100*o)],[ 0 ,  0,  0 ,  1.0]])
#      #desired_position = np.array ([[ 0.627 , -0.471 ,  0.619 ,  1053.44*math.cos((o+10)*math.pi/180)],[-0.46 , -0.86, -0.18 , -28.59*math.sin(o*math.pi/180)],[ 0.62 , -0.17, -0.76, -197.43],[ 0 ,  0,  0 ,  1.0]])
# 
#      #worldToNewTarget = np.matmul(new, toolToNewTarget)
#      new[0,3] = new[0,3] + 10
#      
#       
#      worldToFlangeCommand = np.matmul(new, hR.homInv(flangeToProbeXL143))
#      
#      # move robot according to mode
#      if robotMode == 'SmartServo':
#          
#          poseStringForRobot = hR.make_cmd("MovePTPHomRowWiseMinChangeRT", worldToFlangeCommand)
#          
#      else:
#          
#          poseStringForRobot = hR.make_cmd("MovePTPHomRowWiseMinChange", worldToFlangeCommand)
#              
#      # command pose to robot
#      hR.command(s, BUFFER_SIZE, poseStringForRobot)
#      
#      # wait until position is reached (can be adjusted...)
#      hR.delay(delayBetweenPoses)
# 
#      ## save volumes and robot data
#      
#      timestamp = hR.millis()
#      
#      worldToToolMeas = hR.getWorldToTool(s, BUFFER_SIZE, flangeToProbeXL143)
#      print('worldToToolMeas:  ', worldToToolMeas)
#      
#      jointPos = hR.getJointPos(s, BUFFER_SIZE)
#      print('jointPos:  ', jointPos)
#      
#      jointTorques = hR.getJointTorques(s, BUFFER_SIZE)
#      print('jointTorques:  ', jointTorques)
#      
#      jointExtTorques = hR.getJointTorques(s, BUFFER_SIZE)
#      print('jointExtTorques:  ', jointTorques)
#      
#      forceTorqueEE = hR.getForceTorque(s, BUFFER_SIZE)
#      print('forceTorqueEE', forceTorqueEE)
#      
#      message = client.wait_for_message("Image_Transducer", timeout=3)      
#      
#      dataExp['Timestamp'][countMeas] = timestamp
#      dataExp['ProbePoseCommand'][countMeas] = desired_position
#      dataExp['ProbePoseMeas'][countMeas] = worldToToolMeas
#      dataExp['JointPos'][countMeas] = jointPos
#      dataExp['JointTorques'][countMeas] = jointTorques
#      dataExp['JointExtTorques'][countMeas] = jointExtTorques
#      dataExp['ForceTorqueEE'][countMeas] = forceTorqueEE
#      #dataExp['VolData'][countMeas] = message.image
#      
#      print('Measurement done. Data saved.')
#      
#      countMeas = countMeas + 1
# =============================================================================
     
     



worldToToolend = hR.getWorldToTool(s, BUFFER_SIZE, flangeToProbeXL143)
print('endPoseTool: ', worldToToolend)

s.close()
print("stopped")
           

## save data if wanted. Needs some time, especially volumes need a lot of memory
if saveData:
    date = datetime.now()
    np.save(expName+robotMode+date.strftime("_%d.%m.%Y_%H.%M.%S"), dataExp)












