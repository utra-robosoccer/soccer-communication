#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created June 6 2018

Author: Tyler
"""

import serial
import time
import os
import numpy as np
import struct
from datetime import datetime
from prettytable import PrettyTable

def rxDecoder(raw, decodeHeader=True):
    ''' Decodes raw bytes received from the microcontroller. As per the agreed
        upon protocol, the first 4 bytes are for a header while the remaining
        80 bytes contain floats for each motor.
    '''
    
    motors = list()
    imu = list()
    
    if(decodeHeader):
        header = struct.unpack('<L',raw[0:4])[0]
        for i in range(12):
            # Only reading from 12 motors right now
            motors.append(struct.unpack('<f',raw[8 + i * 4:12 + i * 4])[0])
        for i in range(6):
            # Unpack IMU Data
            imu.append(struct.unpack('<f', raw[56 + i * 4: 60 + i * 4])[0])
        return (header, motors, imu)
    else:
        for i in range(12):
            # Only reading from 12 motors right now
            motors.append(struct.unpack('<f',raw[4 + i * 4:8 + i * 4])[0])
        for i in range(6):
            # Unpack IMU Data
            imu.append(struct.unpack('<f', raw[52 + i * 4: 56 + i * 4])[0])
        return (motors, imu)
    
def logString(userMsg):
    ''' Prints the desired string to the shell, precedded by the date and time.
    '''
    print(datetime.now().strftime('%H.%M.%S.%f') + " " + userMsg)

def sendPacketToMCU(byteStream):
    ''' Sends bytes to the MCU with the header sequence attached.
    '''
    header = struct.pack('<L', 0xFFFFFFFF)
    id = struct.pack('<I', 0x1234)
    padding = bytes(''.encode())
    footer = struct.pack('<L', 0x00000000)
    
    numBytes = len(byteStream)
    if(numBytes < 80):
        padding = struct.pack('<B', 0x00) * (80 - numBytes)
        
    ser.write(header + id + byteStream + padding + footer)
    
def vec2bytes(vec):
    ''' Transforms a numpy vector to a byte array, with entries interpreted as
        32-bit floats.
    '''
    byteArr = bytes(''.encode())
    for element in vec:
        byteArr = byteArr + struct.pack('f', element)
    return byteArr

def printAsAngles(vec1, vec2):
    ''' Prints out 2 numpy vectors side-by-side, where the first vector entry
        is interpreted as belonging to motor 1, the seconds to motor 2, etc.
    '''
    try:
        assert vec1.shape == vec2.shape
    except:
        print("Vec1 shape: {0}\nVec2 shape {1}".format(vec1.shape, vec2.shape))
    
    t = PrettyTable(['Motor Number', 'Sent', 'Received'])
    
    for i in range(vec1.shape[0]):
        t.add_row([str(i + 1), round(vec1[i], 4), round(vec2[i], 2)])
    
    print(t)

def printAsIMUData(vec1):
    ''' Prints out a numpy vector interpreted as data from the IMU, in the
        order X-gyro, Y-gyro, Z-gyro, X-accel, Y-accel, Z-accel.
    '''
    
    t = PrettyTable(['', 'Gyro (deg/s)', 'Accel (m/s^2)'])
    
    t.add_row(["X", round(vec1[0], 2), round(vec1[3], 2)])
    t.add_row(["Y", round(vec1[1], 2), round(vec1[4], 2)])
    t.add_row(["Z", round(vec1[2], 2), round(vec1[5], 2)])
    
    print(t)
    
def receivePacketFromMCU():
    ''' Receives 80 bytes of the MCU provided that there is a valid 4-byte 
        header attached to the front. Returns the list of data interpreted as
        32-bit floats.
    '''
    BUFF_SIZE = 4
    totalBytesRead = 0
    startSeqCount = 0
    buff = bytes(''.encode())
    
    while(True):
        while(ser.in_waiting < BUFF_SIZE):
            time.sleep(0.001)
        rawData = ser.read(BUFF_SIZE)
        
        for i in range(BUFF_SIZE):
            if(startSeqCount == 4):
                buff = buff + rawData[i:i+1]
                totalBytesRead = totalBytesRead + 1
                if(totalBytesRead == 84):
                    break
            else:
                if(struct.unpack('<B', rawData[i:i+1])[0] == 0xFF):
                    startSeqCount = startSeqCount + 1
                else:
                    startSeqCount = 0
        if(totalBytesRead == 84):
            break
    return buff
    
def receiveWithChecks():
    ''' Receives a packet from the MCU and performs basic checks on the packet
        for data integrity. Also decodes the packet and prints a data readout 
        every so often.
    '''
    (recvAngles, recvIMUData) = rxDecoder(receivePacketFromMCU(),
                                            decodeHeader=False)
    
    if(numTransfers % 50 == 0):
        print('\n')
        logString("Received valid data")
        recvAngles = np.array(recvAngles)
        recvAngles = mcuToCtrlAngles(recvAngles)
        printAsAngles(angles[0:12], recvAngles[0:12])
        printAsIMUData(np.array(recvIMUData).reshape((6,)))
    
def ctrlToMcuAngles(ctrlAngles):
    ''' Applies a linear transformation to the motor angles
        received from the control system to convert them to
        the coordinate system used by the motors
    '''
    arr = np.zeros((18,1))
    arr[:ctrlAngles.shape[0],:ctrlAngles.shape[1]] = ctrlAngles
    
    # Multiplicative transformation factor
    m = [1, 1, 1, -1, -1, -1, -1, -1, 1, -1, -1, 1, 1, 1, -1, -1, -1, -1]
    m = np.array(m) * 180.0 / np.pi
    
    # Additive transformation factor
    a = [150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 60, 150, 240, 150, 150]
    a = np.array(a)
    
    return m.dot(arr) + a
    
def mcuToCtrlAngles(mcuAngles):
    ''' Applies a linear transformation to the motor angles
        received from the embedded systems to convert them to
        the coordinate system used by the control systems
    '''
    arr = np.zeros((18,))
    
    arr[:mcuAngles.shape[0],] = mcuAngles
    # Additive transformation factor
    a = [150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 60, 150, 240, 150, 150]
    a = np.array(a)
    
    # Multiplicative transformation factor
    m = [1, 1, 1, -1, -1, -1, -1, -1, 1, -1, -1, 1, 1, 1, -1, -1, -1, -1]
    m = np.array(m) * 180.0 / np.pi
    
    return (arr - a) / m

if __name__ == "__main__":
    logString("Starting PC-side application")
    useTrajectory = False
    stepThrough = False
    useJetson = False
    
    if(useTrajectory):
        if(useJetson):
            os.chdir('/home/nvidia/soccer-embedded/Development/Comm-PC')
        else:
            print("Change to your directory here!")
        trajectory = np.loadtxt(open("getupfront.csv", "rb"), delimiter=",", skiprows=0)
    
    if(useJetson):
        port = '/dev/ttyUSB0'
    else:
        port = 'COM14'
    
    with serial.Serial(port,230400,timeout=100) as ser:
        logString("Opened port " + ser.name)
        
        numTransfers = 0
        while(ser.isOpen()):
            if(useTrajectory):
                for i in range(trajectory.shape[1]):
                    if(stepThrough):
                        dummy = input('Press anything to step through the trajectory')
                    angles = trajectory[:, i:i+1]
                    sendPacketToMCU(vec2bytes(angles))
                    numTransfers = numTransfers + 1
                    receiveWithChecks()
            else:
                if(stepThrough):
                    dummy = input('Press anything to step through the trajectory')
                angles = np.zeros((18,1))
                angles = ctrlToMcuAngles(angles)
                sendPacketToMCU(vec2bytes(angles))
                numTransfers = numTransfers + 1
                receiveWithChecks()
