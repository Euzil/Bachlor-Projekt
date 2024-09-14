# -*- coding: utf-8 -*-
"""
Created on Tue May 11 16:39:36 2021
short script for swtiching between smart servo and handmode by using the keyboard
@author: Jonas
preferences -> run in external system terminal
"""


import math
import numpy as np
import socket
from scipy import io
from scipy.spatial.transform import Rotation as R
import time
import msvcrt # import getch instead for linux --> remove msvcrt.kbhit() if statement
# import ikTest
# import keyboard
#import inversed_kinematics as ik

TCP_IP = '127.0.0.1'#'141.83.19.28'
TCP_PORT = 8000
BUFFER_SIZE = 1024


s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))

s.recv(BUFFER_SIZE)


def command(s, message):
    """
    This function greets to
    the person passed in as
    a parameter
    """
    s.send(message.encode())
    print(s.recv(BUFFER_SIZE))

def make_cmd(root_cmd, pose4x4):
    msg = root_cmd + " {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f}".format(pose4x4[0,0], pose4x4[0,1], \
            pose4x4[0,2], pose4x4[0,3], pose4x4[1,0], pose4x4[1,1], pose4x4[1,2], \
                pose4x4[1,3], pose4x4[2,0], pose4x4[2,1], pose4x4[2,2], pose4x4[2,3], elbowDeg)
    return msg

def make_cmd_from_13(root_cmd, pose1x13):
    msg = root_cmd + " {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f}".format(pose1x13[0,0],pose1x13[0,1], \
            pose1x13[0,2], pose1x13[0,3], pose1x13[0,4], pose1x13[0,5], pose1x13[0,6], \
                pose1x13[0,7], pose1x13[0,8], pose1x13[0,9], pose1x13[0,10], \
                    pose1x13[0,11], pose1x13[0,12])
    return msg

def make_cmd_joints(root_cmd, jointAngles):
    msg = root_cmd + " {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f}".format(jointAngles[0,0], jointAngles[0,1], jointAngles[0,2], jointAngles[0,3], jointAngles[0,4], jointAngles[0,5], jointAngles[0,6])
    return msg

def fillStateArray(stateList):
    # takes the timestamp and measurement strings and put them in one array
    n_meas = len(stateList)
    count = 0

    # first loop to get total number of measured values TODO: mit timestamp: range(1,n_meas)
    for i in range(0,n_meas):
        # get array from list of regarding measurement
        temp = np.array( [np.fromstring(stateList[i].replace('true', '') , sep=" ")], ndmin=2 )
        count = count + np.size(temp,1)

    currentPoseStatesArray = np.empty((1,count))
    # reset count
    ind_start = 0
    ind_add = 0

    # second loop to fill in currentPoseStateArray
    for i in range(0,n_meas):
    # get array from list of regarding measurement
        temp = np.array( [np.fromstring(stateList[i].replace('true', '') , sep=" ")], ndmin=2 )
        ind_add = np.size(temp,1)
        currentPoseStatesArray[ 0,ind_start:(ind_start+ind_add) ] = temp
        ind_start = ind_start + ind_add

    return currentPoseStatesArray


# prepares a rob command
def preparePoseHomRowWise(rawMeasString):

    n_meas = len(rawMeasString)
    measArray = np.zeros(shape=(n_meas,13))

    for i in range(0,n_meas):
        measArray[i,:] = np.array( [np.fromstring(rawMeasString[i].replace('true', '0') , sep=" ")], ndmin=2 )

    return measArray



# prepares a rob command joints
def preparePoseJoints(rawMeasString):

    n_meas = len(rawMeasString)
    measArray = np.zeros(shape=(n_meas,7))

    for i in range(0,n_meas):
        measArray[i,:] = np.array( [np.fromstring(rawMeasString[i].replace('true', '') , sep=" ")], ndmin=2 )

    return measArray

# activate gravity compensation
# =============================================================================
# command(s, "SetBaseRotation 0 0 0")
# command(s, "AT US")
# command(s, "A")
# 
#
# nur SmartServo aktivieren -> so müsste es klappen
command(s, "D")
command(s, "DeactSmartServo")
#command(s, "SetBaseRotation 0 0 0")
command(s, "SetBaseRotation 180 -90 0") # wall mounted iiwa
# command(s, "SetCartStiffness 100 100 100 300 300 300") # very soft impedance to demonstrate compliance
command(s, "SetCartStiffness 5000 5000 5000 300 300 300")
command(s, "SetCartDamping 1 1 1 1 1 1")
#command(s, "SetCartStiffness 1000 1000 5000 300 300 300") # von Sven
command(s, "AT US")

command(s, "SetJntVelRel 0.01")
command(s, "SetJntAccRel 0.1")

command(s, "SetSmartServoJntVelRel 0.1")
command(s, "SetCartVel 0.01")
command(s, "SetSmartServoJntAccelRel 0.1")

command(s, "ActSmartServo") # ggf timerpause zwischen??
# =============================================================================



elbowDeg = 0

isOperational = True
startMeasurement = False
saveMeasurement = False

key = ' '
stateList = []
stateListJoints = []

while isOperational:
    key = " "

    if msvcrt.kbhit():
        key = msvcrt.getwch()
        print(key)

        if key == "e":
            isOperational = False
            print("Key for script ending pressed.")
            break

        # activate gravity compensation
        if key == "h":
            handmodeOn = True
            print("Handmode turned on.")
            command(s, "DeactSmartServo")
            command(s, "AT US")
            command(s, "ActGravCompNoDiag")
            
            key = ' '
            #time.sleep(10)
            #key = "o"

        if key == "r":
            stateList = []
            stateListJoints = []
            key = ' '
            print("Recording turned on.")

            startMeasurement = True
            #time.sleep(10)
            #key = "o"

        # turn back to smart servo mode
        if key == "o":
            handmodeOn = False
            print("Handmode turned off. Switching back to Smart Servo Mode.")
            command(s, "D")
            command(s, "ActSmartServo")
            key = ' '
            startMeasurement = False
            saveMeasurement = True


       # if key == "q":
           # command(s, "DeactSmartServo")
            # calculate and command the pilotscan path
          #  key == ' '

           # savedStateList = np.load('measurementsStringPoseHomRowWise_SCAN.npy')
           # time.sleep(0.5)

          #  posesHomRowWise = preparePoseHomRowWise(savedStateList)

            # go to first pose of pilotscan
          #  poseStringForRobot = make_cmd_from_13("MovePTPHomRowWiseMinChangeRT", posesHomRowWise[0:1,:])
           # command(s,poseStringForRobot)
          #  time.sleep(10)

            # command path of scanned poses
         #   for i in range(0, len(posesHomRowWise)-1, 100):
          #      poseStringForRobot = make_cmd_from_13("MovePTPHomRowWiseMinChangeRT", posesHomRowWise[i:i+1,:])
            #    command(s,poseStringForRobot)
            #    time.sleep(0.05)

        # klappt ganz gut so mit 1000 step und 0.01 vel und 0.1 accel
        if key == "m":
            #hcommand(s, "DeactSmartServo")
            key == ' '
            #savedStateListJoints = np.load('measurementsStringJoints_SCAN.npy')
            savedStateListJoints = np.load('measurementsStringJoints_testscan.npy')

            time.sleep(0.5)

            jointAngles = preparePoseJoints(savedStateListJoints)

           # poseStringForRobot = make_cmd_joints("SmartServoSetJoints", jointAngles[0:1,:])
          #  poseStringForRobot = make_cmd_joints("MovePTPJoints", jointAngles[0:1,:])
           # command(s,poseStringForRobot)
           # time.sleep(5)
            key = ''
            while True: # motion replay loop while waiting for abort key press


                for i in range(len(jointAngles)-1,0,-1): # start motion replay in reverse direction
                   # poseStringForRobot = make_cmd_joints("SmartServoSetJoints", jointAngles[i:i+1,:])
                    poseStringForRobot = make_cmd_joints("SmartServoSetJoints", jointAngles[i:i+1,:])
                    command(s,poseStringForRobot)
                    #time.sleep(0.05) # possibly unnecessary with direct ethernet connection between PC and robot
                    if msvcrt.kbhit():
                        key = msvcrt.getwch()
                        print(key)
                        break
                if key != '':
                    break

                for i in range(0, len(jointAngles) - 1):  
                    # poseStringForRobot = make_cmd_joints("SmartServoSetJoints", jointAngles[i:i+1,:])
                    poseStringForRobot = make_cmd_joints("SmartServoSetJoints", jointAngles[i:i + 1, :])
                    command(s, poseStringForRobot)
                    if msvcrt.kbhit():
                        key = msvcrt.getwch()
                        print(key)
                        break
                    # time.sleep(0.05)# possibly unnecessary with direct ethernet connection between PC and robot
                if key != '':
                    break

    if startMeasurement == True:
        # s.send(("GetPositionHomRowWise").encode())
        # output = (s.recv(BUFFER_SIZE)).decode()
        # stateList.append(output)                ####  comment to reduce delay for retrieval of joint positions

        s.send(("GetPositionJoints").encode())
        output = (s.recv(BUFFER_SIZE)).decode()
        stateListJoints.append(output)


    if saveMeasurement == True:
        # np.save('measurementsStringPoseHomRowWise_testscan', stateList)
        np.save('measurementsStringJoints_testscan', stateListJoints)
        saveMeasurement = False


print("stopped")



