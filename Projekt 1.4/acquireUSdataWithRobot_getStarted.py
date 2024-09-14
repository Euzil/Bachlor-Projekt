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
    new[0, 3] = new[0, 3] + rangex

    worldToFlangeCommand = np.matmul(new, hR.homInv(flangeToProbeXL143))

    # move robot according to mode
    if robotMode == 'SmartServo':

        poseStringForRobot = hR.make_cmd(
            "MovePTPHomRowWiseMinChangeRT", worldToFlangeCommand)

    else:

        poseStringForRobot = hR.make_cmd(
            "MovePTPHomRowWiseMinChange", worldToFlangeCommand)

    # command pose to robot
    hR.command(s, BUFFER_SIZE, poseStringForRobot)

    # wait until position is reached (can be adjusted...)
    hR.delay(delayBetweenPoses)

    # save volumes and robot data

    timestamp = hR.millis()

    worldToToolMeas = hR.getWorldToTool(s, BUFFER_SIZE, flangeToProbeXL143)
    # print('worldToToolMeas:  ', worldToToolMeas)

    jointPos = hR.getJointPos(s, BUFFER_SIZE)
    # print('jointPos:  ', jointPos)

    jointTorques = hR.getJointTorques(s, BUFFER_SIZE)
    # print('jointTorques:  ', jointTorques)

    jointExtTorques = hR.getJointTorques(s, BUFFER_SIZE)
    # print('jointExtTorques:  ', jointTorques)

    forceTorqueEE = hR.getForceTorque(s, BUFFER_SIZE)
    # print('forceTorqueEE', forceTorqueEE)


def movey(rangey):

    new = hR.getWorldToTool(s, BUFFER_SIZE, flangeToProbeXL143)
    new[1, 3] = new[1, 3] + rangey

    worldToFlangeCommand = np.matmul(new, hR.homInv(flangeToProbeXL143))

    # move robot according to mode
    if robotMode == 'SmartServo':

        poseStringForRobot = hR.make_cmd(
            "MovePTPHomRowWiseMinChangeRT", worldToFlangeCommand)

    else:

        poseStringForRobot = hR.make_cmd(
            "MovePTPHomRowWiseMinChange", worldToFlangeCommand)

    # command pose to robot
    hR.command(s, BUFFER_SIZE, poseStringForRobot)

    # wait until position is reached (can be adjusted...)
    hR.delay(delayBetweenPoses)

    # save volumes and robot data

    timestamp = hR.millis()

    worldToToolMeas = hR.getWorldToTool(s, BUFFER_SIZE, flangeToProbeXL143)
    # print('worldToToolMeas:  ', worldToToolMeas)

    jointPos = hR.getJointPos(s, BUFFER_SIZE)
    # print('jointPos:  ', jointPos)

    jointTorques = hR.getJointTorques(s, BUFFER_SIZE)
    # print('jointTorques:  ', jointTorques)

    jointExtTorques = hR.getJointTorques(s, BUFFER_SIZE)
    # print('jointExtTorques:  ', jointTorques)

    forceTorqueEE = hR.getForceTorque(s, BUFFER_SIZE)
    # print('forceTorqueEE', forceTorqueEE)


def movez(rangez):

    new = hR.getWorldToTool(s, BUFFER_SIZE, flangeToProbeXL143)
    new[2, 3] = new[2, 3] + rangez

    worldToFlangeCommand = np.matmul(new, hR.homInv(flangeToProbeXL143))

    # move robot according to mode
    if robotMode == 'SmartServo':

        poseStringForRobot = hR.make_cmd(
            "MovePTPHomRowWiseMinChangeRT", worldToFlangeCommand)

    else:

        poseStringForRobot = hR.make_cmd(
            "MovePTPHomRowWiseMinChange", worldToFlangeCommand)

    # command pose to robot
    hR.command(s, BUFFER_SIZE, poseStringForRobot)

    # wait until position is reached (can be adjusted...)
    hR.delay(delayBetweenPoses)

    # save volumes and robot data

    timestamp = hR.millis()

    worldToToolMeas = hR.getWorldToTool(s, BUFFER_SIZE, flangeToProbeXL143)
    # print('worldToToolMeas:  ', worldToToolMeas)

    jointPos = hR.getJointPos(s, BUFFER_SIZE)
    # print('jointPos:  ', jointPos)

    jointTorques = hR.getJointTorques(s, BUFFER_SIZE)
    # print('jointTorques:  ', jointTorques)

    jointExtTorques = hR.getJointTorques(s, BUFFER_SIZE)
    # print('jointExtTorques:  ', jointTorques)

    forceTorqueEE = hR.getForceTorque(s, BUFFER_SIZE)
    # print('forceTorqueEE', forceTorqueEE)


def initialize():

    desired_position = np.array([[0.627, 0.-4.71,  0.619,  1053.44], [-0.46, -
                                0.86, -0.18, -28.59], [0.62, -0.17, -0.76, -197.43], [0,  0,  0,  1.0]])
    # desired_position = np.array ([[-0.33, -0.77,  0.54,  1141.65],[-0.53, -0.316, -0.79, -81],  [ 0.77, -0.55, -0.3, -290.77], [0 ,0 , 0, 1]])

    # move robot according to mode
    if robotMode == 'SmartServo':

        poseStringForRobot = hR.make_cmd(
            "MovePTPHomRowWiseMinChangeRT", desired_position)

    else:

        poseStringForRobot = hR.make_cmd(
            "MovePTPHomRowWiseMinChange", desired_position)

        # command pose to robot
    hR.command(s, BUFFER_SIZE, poseStringForRobot)

    # wait until position is reached (can be adjusted...)
    hR.delay(delayBetweenPoses)

    # save volumes and robot data

    timestamp = hR.millis()

    worldToToolMeas = hR.getWorldToTool(s, BUFFER_SIZE, flangeToProbeXL143)
    # print('worldToToolMeas:  ', worldToToolMeas)

    jointPos = hR.getJointPos(s, BUFFER_SIZE)
    # print('jointPos:  ', jointPos)

    jointTorques = hR.getJointTorques(s, BUFFER_SIZE)
    # print('jointTorques:  ', jointTorques)

    jointExtTorques = hR.getJointTorques(s, BUFFER_SIZE)
    # print('jointExtTorques:  ', jointTorques)

    forceTorqueEE = hR.getForceTorque(s, BUFFER_SIZE)
    # print('forceTorqueEE', forceTorqueEE)

    print('Start position reached')

# set the position and direction


def set():
    # start_position = np.array ([[ 0.627 , 0.-4.71 ,  0.627 ,  x ],[-0.471 , -0.86, -0.18 , y],[ 0.627 , -0.18, -0.86, z],[ 0 ,  0,  0 ,  1.0]])
    # start_position = np.array ([[ 0.6 , 0.-6 ,  0.6 ,  1000 ],[-0.5 , -0.5, -0.5 , 0],[ 0.7 , -0.2, -0.8, 0],[ 0 ,  0,  0 ,  1.0]])
    # start_position = np.array ([[ 0.707 , 0.0 ,  0.707 ,  1000.0 ],[0.0, 1.0, 0.0 , 0.0],[ -0.707 , 0.0, 0.707, 0.0],[ 0 ,  0,  0 ,  1.0]])
    start_position = np.array([[-0.707, 0.0,  0.707,  1000.0], [0.0,
                              1.0, 0.0, 0.0], [-0.707, 0.0, -0.707, 0.0], [0,  0,  0,  1.0]])
    # move robot according to mode
    if robotMode == 'SmartServo':

        poseStringForRobot = hR.make_cmd(
            "MovePTPHomRowWiseMinChangeRT", start_position)

    else:

        poseStringForRobot = hR.make_cmd(
            "MovePTPHomRowWiseMinChange", start_position)

        # command pose to robot
    hR.command(s, BUFFER_SIZE, poseStringForRobot)

    # wait until position is reached (can be adjusted...)
    hR.delay(delayBetweenPoses)

    # save volumes and robot data

    timestamp = hR.millis()

    worldToToolMeas = hR.getWorldToTool(s, BUFFER_SIZE, flangeToProbeXL143)
    # print('worldToToolMeas:  ', worldToToolMeas)

    jointPos = hR.getJointPos(s, BUFFER_SIZE)
    # print('jointPos:  ', jointPos)

    jointTorques = hR.getJointTorques(s, BUFFER_SIZE)
    print('jointTorques:  ', jointTorques)

    jointExtTorques = hR.getJointTorques(s, BUFFER_SIZE)
    print('jointExtTorques:  ', jointTorques)

    forceTorqueEE = hR.getForceTorque(s, BUFFER_SIZE)
    # print('forceTorqueEE', forceTorqueEE)

    print('Position reached')


def incline(degrees):
    new = hR.getWorldToTool(s, BUFFER_SIZE, flangeToProbeXL143)
    radian = math.radians(degrees)
    # around y axis
    # Matrix Multiplication
    result = np.matmul(new, hR.rotyHMD(degrees))
    # change the Matrix
    new[0, 0] = new[0, 0] + math.cos(radian)
    new[0, 2] = new[0, 2] + math.sin(radian)
    new[2, 0] = new[2, 0] - math.sin(radian)
    new[2, 2] = new[2, 2] + math.cos(radian)

    worldToFlangeCommand = np.matmul(result, hR.homInv(flangeToProbeXL143))

    # move robot according to mode
    if robotMode == 'SmartServo':

        poseStringForRobot = hR.make_cmd(
            "MovePTPHomRowWiseMinChangeRT", worldToFlangeCommand)

    else:

        poseStringForRobot = hR.make_cmd(
            "MovePTPHomRowWiseMinChange", worldToFlangeCommand)

    # command pose to robot
    hR.command(s, BUFFER_SIZE, poseStringForRobot)

    # wait until position is reached (can be adjusted...)
    hR.delay(delayBetweenPoses)

    # save volumes and robot data

    timestamp = hR.millis()

    worldToToolMeas = hR.getWorldToTool(s, BUFFER_SIZE, flangeToProbeXL143)
    # print('worldToToolMeas:  ', worldToToolMeas)

    jointPos = hR.getJointPos(s, BUFFER_SIZE)
    # print('jointPos:  ', jointPos)

    jointTorques = hR.getJointTorques(s, BUFFER_SIZE)
    # print('jointTorques:  ', jointTorques)

    jointExtTorques = hR.getJointTorques(s, BUFFER_SIZE)
    # print('jointExtTorques:  ', jointTorques)

    forceTorqueEE = hR.getForceTorque(s, BUFFER_SIZE)
    # print('forceTorqueEE', forceTorqueEE)

    print('finish incline')


def circle(deg):
    new = hR.getWorldToTool(s, BUFFER_SIZE, flangeToProbeXL143)
    basis = hR.getWorldToTool(s, BUFFER_SIZE, flangeToProbeXL143)
    r = 1
    i = 1
    while i < deg:
        radian = math.radians(1)
        new = np.matmul(new, hR.rotzHMD(1))
        new[1, 3] = basis[1, 3] + r*math.sin(radian)
        new[0, 3] = basis[0, 3] + r - r*math.cos(radian)
        worldToFlangeCommand = np.matmul(new, hR.homInv(flangeToProbeXL143))
        # move robot according to mode
        if robotMode == 'SmartServo':
            poseStringForRobot = hR.make_cmd(
                "MovePTPHomRowWiseMinChangeRT", worldToFlangeCommand)
        else:
            poseStringForRobot = hR.make_cmd(
                "MovePTPHomRowWiseMinChange", worldToFlangeCommand)
        # command pose to robot
        hR.command(s, BUFFER_SIZE, poseStringForRobot)
        print("cirile in deg :", i)
        # wait until position is reached (can be adjusted...)
        hR.delay(delayBetweenPoses)
        # save volumes and robot data
        timestamp = hR.millis()
        i = i+1

    worldToToolMeas = hR.getWorldToTool(s, BUFFER_SIZE, flangeToProbeXL143)
    # print('worldToToolMeas:  ', worldToToolMeas)

    jointPos = hR.getJointPos(s, BUFFER_SIZE)
    # print('jointPos:  ', jointPos)

    jointTorques = hR.getJointTorques(s, BUFFER_SIZE)
    # print('jointTorques:  ', jointTorques)

    jointExtTorques = hR.getJointTorques(s, BUFFER_SIZE)
    # print('jointExtTorques:  ', jointTorques)

    forceTorqueEE = hR.getForceTorque(s, BUFFER_SIZE)
    # print('forceTorqueEE', forceTorqueEE)


def rotate(x, y):
    new = hR.getWorldToTool(s, BUFFER_SIZE, flangeToProbeXL143)
    radianY = math.radians(x)
    radianZ = math.radians(y)
    # around y axis
    # Martix multiplication
    resultR = np.matmul(hR.rotyHMD(x), hR.rotzHMD(y))
    result = np.matmul(new, resultR)
    # change the Martix
    new[0, 0] = new[0, 0] + math.cos(radianY)*math.cos(radianZ)
    new[0, 1] = new[0, 1] - math.cos(radianY)*math.sin(radianZ)
    new[0, 2] = new[0, 2] + math.sin(radianY)

    new[1, 0] = new[1, 0] + math.sin(radianZ)
    new[1, 1] = new[1, 1] + math.cos(radianZ)

    new[2, 0] = new[2, 0] - math.sin(radianY)*math.cos(radianZ)
    new[2, 1] = new[2, 1] + math.sin(radianY)*math.sin(radianZ)
    new[2, 2] = new[2, 2] + math.cos(radianY)
    print("roooootttttaaaation")

    worldToFlangeCommand = np.matmul(result, hR.homInv(flangeToProbeXL143))

    # move robot according to mode
    if robotMode == 'SmartServo':

        poseStringForRobot = hR.make_cmd(
            "MovePTPHomRowWiseMinChangeRT", worldToFlangeCommand)

    else:

        poseStringForRobot = hR.make_cmd(
            "MovePTPHomRowWiseMinChange", worldToFlangeCommand)

    # command pose to robot
    hR.command(s, BUFFER_SIZE, poseStringForRobot)

    # wait until position is reached (can be adjusted...)
    hR.delay(delayBetweenPoses)

    # save volumes and robot data

    timestamp = hR.millis()

    worldToToolMeas = hR.getWorldToTool(s, BUFFER_SIZE, flangeToProbeXL143)
    # print('worldToToolMeas:  ', worldToToolMeas)

    jointPos = hR.getJointPos(s, BUFFER_SIZE)
    # print('jointPos:  ', jointPos)

    jointTorques = hR.getJointTorques(s, BUFFER_SIZE)
    # print('jointTorques:  ', jointTorques)

    jointExtTorques = hR.getJointTorques(s, BUFFER_SIZE)
    # print('jointExtTorques:  ', jointTorques)

    forceTorqueEE = hR.getForceTorque(s, BUFFER_SIZE)
    # print('forceTorqueEE', forceTorqueEE)

    print(' finish rotation')


# set up port for robot connection
TCP_IP = '127.0.0.1'
TCP_PORT = 8000
BUFFER_SIZE = 1024

# transformation of mounted ultrasound probe, here the XL143
# WARNING: has to be adapted if another probe is used! Make sure that
# the probe holder is monted in accordance with the transformation matrix.

flangeToProbeXL143 = np.array(
    [[0.7071, 0, 0.7071, 30], [0, 1, 0, 0], [-0.7071, 0, 0.7071, 135], [0, 0, 0, 1]])
# connect and initialize robot
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))
s.recv(BUFFER_SIZE)

robotMode = 'DirectServo'  # DirectServo (normal mode) or SmartServo

# initialize robot parameters
# go to helperRobot file to adapt initialization parameters (velocities, damping, ...)
hR.initRobot(s, BUFFER_SIZE, robotMode)

# open IGTLinkClient, PLUS server sends images
client = pyigtl.OpenIGTLinkClient(host="127.0.0.1", port=18944)

# get initial image from US station to define shape
message = client.wait_for_message("Image_Transducer", timeout=3)
# volShape = np.shape(message.image)


# define data struct to save measurements
tp = np.dtype([('Timestamp', 'f8'), ('ProbePoseCommand', 'f8', (4, 4)),
               ('ProbePoseMeas', 'f8', (4, 4)), ('JointPos', 'f8', (1, 7)),
               ('JointTorques', 'f8', (1, 7)), ('JointExtTorques', 'f8', (1, 7)),
               ('ForceTorqueEE', 'f8', (1, 6))])


# counter to fill measurements data array
countMeas = 0

# total number of measurements, which should be saved.
# TODO: This should be improved...
totNumbMeas = 10

# save data in struct array
dataExp = np.zeros(totNumbMeas, dtype=tp)

# name for experiment
expName = 'TransProbeMovement'
saveData = False


# get current pose of the probe as startpose for the movement
worldToToolStart = hR.getWorldToTool(s, BUFFER_SIZE, flangeToProbeXL143)

print('startPoseTool: ', worldToToolStart)

# wait 5 s
hR.delay(5000)

# might be useful in SmartServo mode to ensure that every pose is reached before
# saving the robot stat and the us images.
delayBetweenPoses = 500  # in ms 1500


# vertical move

# set a start position and correct in vertical

set()
#movex(60)
movey(-200)
# move along the z direction
Lang = int(input("move strength of World in z direction :"))
Lang2 = int(input("move strength of World in x direction :"))

Kraft = np.sum(hR.getExtJointTorques(s, BUFFER_SIZE))

angle =int(input("incline in degress :"))
rund =int(input("circle in degress :"))
incline(angle)
#circle(rund)
# movex(50)
# movex(-50)


# rotate(0,120)

collision = False
no_collsion = True
tresh = np.sum(hR.getExtJointTorques(s, BUFFER_SIZE))
i = 0

while i < Lang and no_collsion:
    Force = np.sum(hR.getExtJointTorques(s, BUFFER_SIZE))
    movez(-1)
    print('SUM FORCE:', Force)
    if Force > tresh+0.4:
        distance = i
        print('distance:', distance)
        movez(1)
        print("---emergency stop---")
        collision = True
        no_collsion = False
        print('collision:', collision)
        tresh = np.sum(hR.getExtJointTorques(s, BUFFER_SIZE))
    print("i :", i)
    i = i+1

i = 0
if collision == True:
    while i < Lang2:
        Force = np.sum(hR.getExtJointTorques(s, BUFFER_SIZE))
        tresh = np.sum(hR.getExtJointTorques(s, BUFFER_SIZE))
        print('Force:',Force ,'tresh',tresh)
        '''
        if Force < 0:
            movez(-1)
        if Force >1:
            movez(1)
        '''
        movex(1)
        print('+i:', i)
        i = i+1
    i = 0
    while i < Lang2:
        Force = np.sum(hR.getExtJointTorques(s, BUFFER_SIZE))
        tresh = np.sum(hR.getExtJointTorques(s, BUFFER_SIZE))
        print('Force:',Force ,'tresh',tresh)

        '''
        if Force < 0:
            movez(-1)
        if Force > 1:
            movez(1)
        '''
        movex(-1)
        i = i+1
        print('-i:', i)

print('distance:', distance)
movez(-distance)

print('fertig')


'''
#toach emergency stop
i=0
collision = None

while i < 2*Lang:   

    delayBetweenPoses = 0
    Force = np.sum(hR.getExtJointTorques(s, BUFFER_SIZE))
     
    if Force > 0: 

        collision = True
        movez(0)
        i=0
        #Treshhorld=np.sum(hR.getForceTorque(s, BUFFER_SIZE))     
        print('FORCE:', Force,'COLLISION:',collision,"i :",i)
        print("---emergency stop---")
        
    else:

        collision = False
        movez(-0.5)
        print('FORCE:', Force,'COLLISION:',collision,"i :",i)
        
    i=i+1   

'''
'''
# incline in degrees
angle =int(input("incline in degress"))
incline(-angle)

movex(50)
movex(-50)
movex(50)
movex(-50)
movex(50)
movex(-50)
movex(50)
movex(-50)
movex(50)
movex(-50)
movex(50)
movex(-50)

'''


worldToToolend = hR.getWorldToTool(s, BUFFER_SIZE, flangeToProbeXL143)
print('endPoseTool: ', worldToToolend)

s.close()
print("stopped")


# save data if wanted. Needs some time, especially volumes need a lot of memory
if saveData:
    date = datetime.now()
    np.save(expName+robotMode+date.strftime("_%d.%m.%Y_%H.%M.%S"), dataExp)
