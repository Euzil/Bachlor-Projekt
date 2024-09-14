# -*- coding: utf-8 -*-
"""
Created on Mon Jan 10 11:17:08 2022
helper functions for the robot
@author: Jonas
"""
import numpy as np
import math
import ctypes, os 
import scipy.ndimage as ndi 
from scipy import integrate


def initRobot(s, bufferSize, robotMode):
    """
    initializes robot and activates SmartServo Mode if wanted
    """   
    
    # first deactivate and activate again to be able to change stiffness
    command(s, bufferSize, "DeactSmartServo")
    
    command(s, bufferSize, "SetBaseRotation 180 -90 0")
    command(s, bufferSize, "SetCartStiffness 3000 3000 3000 200 200 200")
    # bis 5000/300 setzbar
    #command(s, "SetCartStiffness 1000 1000 5000 300 300 300") # von Sven
    command(s, bufferSize, "AT US")

    command(s, bufferSize, "SetJntVelRel 0.1")
    command(s, bufferSize, "SetJntAccRel 0.1")

    command(s, bufferSize, "SetSmartServoJntVelRel 0.1")
    command(s, bufferSize, "SetSmartServoJntAccelRel 0.1")
    
    command(s, bufferSize, "SetCartVel 0.1") # any influence of this command??
    
    if robotMode == 'SmartServo':
        command(s, bufferSize, "ActSmartServo")
    
       
    
def command(s, bufferSize, message):
    """
    This function greets to
    the person passed in as
    a parameter
    """
    s.send(message.encode())
    print(s.recv(bufferSize))


def make_cmd(root_cmd, pose4x4):
    elbowDeg = 3.0
    msg = root_cmd + " {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f}".format(pose4x4[0,0], pose4x4[0,1], \
            pose4x4[0,2], pose4x4[0,3], pose4x4[1,0], pose4x4[1,1], pose4x4[1,2], \
                pose4x4[1,3], pose4x4[2,0], pose4x4[2,1], pose4x4[2,2], pose4x4[2,3], elbowDeg)
    return msg



def getWorldToFlange(s,bufferSize):
    'get flange pose of robot and transform it to a homogenous matrix'
    
    s.send(("GetPositionHomRowWise").encode())
    rawMeasString = (s.recv(bufferSize)).decode()
    
    measArray = np.zeros(shape=(1,13))
    measArray[0,:] = np.array( [np.fromstring(rawMeasString.replace('true', '0') , sep=" ")], ndmin=2 )
    
    worldToFlange = np.identity(4)
    worldToFlange[0:3,0:4] = np.reshape(measArray[0,:-1], (3,4))
    
    return worldToFlange


   
def getWorldToTool(s, bufferSize, flangeToTool):
    'get tool pose of robot and transform it to a homogenous matrix'
    
    worldToTool = np.identity(4)
    worldToFlange = getWorldToFlange(s, bufferSize)
    
    worldToTool = np.matmul(worldToFlange, flangeToTool)
    
    return worldToTool


def getJointPos(s, bufferSize):
    'get joint positions from robot'
    
    s.send(("GetPositionJoints").encode())
    rawMeasString = (s.recv(bufferSize)).decode()
    
    jointPos = np.array( [np.fromstring(rawMeasString.replace('true', '') , sep=" ")], ndmin=2 )
        
    
    return jointPos


def getJointTorques(s, bufferSize):
    'get joint torques from robot'
    
    s.send(("GetJointTorques").encode())
    rawMeasString = (s.recv(bufferSize)).decode()
    
    jointTorques = np.array( [np.fromstring(rawMeasString.replace('true', '') , sep=" ")], ndmin=2 )
    
    return jointTorques    


def getExtJointTorques(s, bufferSize):
    'get joint torques from robot'
    
    s.send(("GetExtJointTorques").encode())
    rawMeasString = (s.recv(bufferSize)).decode()
    
    jointExtTorques = np.array( [np.fromstring(rawMeasString.replace('true', '') , sep=" ")], ndmin=2 )    
    
    return jointExtTorques    




def getForceTorque(s, bufferSize):
    'get end effector wrnech from robot'
    
    s.send(("GetForceTorque").encode())
    rawMeasString = (s.recv(bufferSize)).decode()
    
    forceTorqueEE = np.array( [np.fromstring(rawMeasString.replace('true', '') , sep=" ")], ndmin=2 )
    
    return forceTorqueEE



# rotations as homogenous matrix in deg
def rotxHMD(angleDeg):
    rotx = np.array([[1,0,0,0],[0,math.cos(angleDeg*math.pi/180),-math.sin(angleDeg*math.pi/180),0],
                     [0, math.sin(angleDeg*math.pi/180),math.cos(angleDeg*math.pi/180),0], [0,0,0,1] ])
    return rotx

def rotyHMD(angleDeg):
    roty=np.array([[math.cos(angleDeg*math.pi/180), 0, math.sin(angleDeg*math.pi/180), 0],[0,1,0,0],
                   [-math.sin(angleDeg*math.pi/180), 0, math.cos(angleDeg*math.pi/180),0], [0,0,0,1]])
    return roty

def rotzHMD(angleDeg):
    rotz=np.array([[math.cos(angleDeg*math.pi/180), -math.sin(angleDeg*math.pi/180), 0,0], 
                   [math.sin(angleDeg*math.pi/180), math.cos(angleDeg*math.pi/180), 0,0],[0,0,1,0], [0,0,0,1]])
    return rotz

# homogenous inverse
def homInv(matrix):
    hI = np.identity(4)
    hI[0:3, 0:3] = np.transpose(matrix[0:3,0:3])
    hI[0:3,3:4] = -np.matmul(hI[0:3, 0:3] , matrix[0:3,3:4])
    
    return hI

# micro- and millisecond-resolution timstamps
# https://stackoverflow.com/questions/38319606/how-can-i-get-millisecond-and-microsecond-resolution-timestamps-in-python
def micros():
    "return a timestamp in microseconds (us)"
    tics = ctypes.c_int64()
    freq = ctypes.c_int64()

    #get ticks on the internal ~2MHz QPC clock
    ctypes.windll.Kernel32.QueryPerformanceCounter(ctypes.byref(tics)) 
    #get the actual freq. of the internal ~2MHz QPC clock
    ctypes.windll.Kernel32.QueryPerformanceFrequency(ctypes.byref(freq))  
    
    t_us = tics.value*1e6/freq.value
    return t_us
        
def millis():
    "return a timestamp in milliseconds (ms)"
    tics = ctypes.c_int64()
    freq = ctypes.c_int64()

    #get ticks on the internal ~2MHz QPC clock
    ctypes.windll.Kernel32.QueryPerformanceCounter(ctypes.byref(tics)) 
    #get the actual freq. of the internal ~2MHz QPC clock 
    ctypes.windll.Kernel32.QueryPerformanceFrequency(ctypes.byref(freq)) 
    
    t_ms = tics.value*1e3/freq.value
    return t_ms


#Other timing functions:
def delay(delay_ms):
    "delay for delay_ms milliseconds (ms)"
    t_start = millis()
    while (millis() - t_start < delay_ms):
      pass #do nothing 
    return

def delayMicroseconds(delay_us):
    "delay for delay_us microseconds (us)"
    t_start = micros()
    while (micros() - t_start < delay_us):
      pass #do nothing 
    return 

