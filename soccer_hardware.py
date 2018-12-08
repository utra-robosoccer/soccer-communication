#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created June 6 2018

Author: Tyler and Jason
"""

import argparse
import serial
import time
import os
import sys
import numpy as np
import struct

from datetime import datetime
from prettytable import PrettyTable

has_ros = True
try:
    import rospy
    from soccer_msgs.msg import RobotGoal
    from soccer_msgs.msg import RobotState
    from sensor_msgs.msg import Imu
    from geometry_msgs.msg import Vector3
    from geometry_msgs.msg import Quaternion
    from tf.msg import tfMessage
    from tf.transformations import quaternion_from_euler
    from transformations import *
except:
    has_ros = False
    print("No ROS")

def rxDecoder(raw):
    ''' Decodes raw bytes received from the microcontroller. As per the agreed
        upon protocol, the first 4 bytes are for a header while the remaining
        80 bytes contain floats for each motor.
    '''
    motors = list()
    imu = list()

    for i in range(12):
        # Here, we only unpack for 12 motors since that's all we have connected
        # in our current setup
        motors.append(struct.unpack('<f',raw[4 + i * 4:8 + i * 4])[0])
    for i in range(6):
        # Unpack IMU Data
        imu.append(struct.unpack('<f', raw[52 + i * 4: 56 + i * 4])[0])
    return (motors, imu)
    
def logString(userMsg):
    ''' Prints the desired string to the shell, precedded by the date and time.
    '''
    print(datetime.now().strftime('%H.%M.%S.%f') + " " + userMsg)

def sendPacketToMCU(ser, byteStream):
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
    assert vec1.shape[0] == vec2.shape[0]
    t = PrettyTable(['Motor Number', 'Sent', 'Received'])
    
    for i in range(vec1.shape[0]):
        t.add_row([str(i + 1), round(vec1[i][0], 4), round(vec2[i][0], 2)])
    
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
    
def receivePacketFromMCU(ser):
    ''' Receives 80 bytes of the MCU provided that there is a valid 4-byte 
        header attached to the front. Returns the list of data interpreted as
        32-bit floats.
    '''
    
    receive_succeeded = False
    
    totalBytesRead = 0
    startSeqCount = 0
    buff = bytes(''.encode())
    
    timeout = 0.01 # 10 ms timeout
    time_start = time.time()
    time_curr = time_start
    while(True):
        # First, we wait until we have received some data, or until the timeout
        # has elapsed
        while((num_bytes_available == 0) and (time_curr - time.start < timeout)):
            time.sleep(0.001)
            time_curr = time.time()
            num_bytes_available = ser.in_waiting
        
        if((time_curr - time.start) >= timeout):
            # Treat timeout as hard requirement
            break
        else:
            # If we receive some data, we process it here then go back to
            # waiting for more
            rawData = ser.read(num_bytes_available)
            
            for i in range(num_bytes_available):
                if(startSeqCount == 4):
                    buff = buff + rawData[i:i+1]
                    totalBytesRead = totalBytesRead + 1
                    
                    if(totalBytesRead == 84):
                        # If we get here, we have received a full packet
                        receive_succeeded = True
                        break
                else:
                    if(struct.unpack('<B', rawData[i:i+1])[0] == 0xFF):
                        startSeqCount = startSeqCount + 1
                    else:
                        startSeqCount = 0
            
    return (receive_succeeded, buff)

def get_script_path():
    return os.path.dirname(os.path.realpath(sys.argv[0]))
    
class soccer_hardware:
    def __init__(self):
        os.chdir(get_script_path())
        logString("Starting PC-side application")
        
        parser = argparse.ArgumentParser(description='Soccer hardware')
        parser.add_argument(
            '-r',
            '--ros',
            help='Imports ROS-related dependencies if True or omitted. Default: '
                 'True if you have ROS installed, otherwise false',
            default=True
        )
        parser.add_argument(
            '--port',
            help='Specifies the port argument to the serial.Serial constructor. Default: /dev/ttyUSB0',
            default='/dev/ttyACM0'
        )
        
        parser.add_argument(
            '--baud',
            help='Specifies the serial port baud rate. Default: 230400',
            default=230400
        )
        
        parser.add_argument(
            '--traj',
            help='Specifies the trajectory to use by default. Default: standing.csv',
            default='standing.csv'
        )
        
        parser.add_argument(
            '__name',
            nargs='?',
            help='ROS argument'
        )
        
        parser.add_argument(
            '__log',
            nargs='?',
            help='ROS argument'
        )
        
        args = vars(parser.parse_args())
        
        self.isROSmode = args['ros'] and has_ros
        self.port = args['port']
        self.baud = args['baud']
        self.traj = args['traj']
        self.num_transmissions = 0
        self.num_receptions = 0
        self.last_print_time = time.time()
        
        logString("Started with ROS = {0}".format(self.isROSmode))
        logString("Attempting to open trajectory file \'{0}\'".format(self.traj))
        trajectories_dir = os.path.join("trajectories", self.traj)
        
        try:
            self.trajectory = np.loadtxt(
                open(trajectories_dir, "rb"),
                delimiter=",",
                skiprows=0
            )
            
            logString("Initialized soccer hardware with trajectory {0}".format(
                    self.traj
                )
            )
        except FileNotFoundError as err:
            logString("Could not open trajectory: {0}. Is your Python shell "
                      "running in the soccer-communication directory?".format(err))
                
    def connect_to_embedded(self):
        logString(
            "Attempting connection to embedded systems via port {0} with baud rate {1}".format(
                self.port,
                self.baud
            )
        )
        
        num_tries = 0
        while(1):
            try:
                self.ser = serial.Serial(self.port, self.baud,timeout=100)
                self.ser.open()
            except:
                if(num_tries % 100 == 0):
                    logString("Connection failed. Retrying...(attempt {0})".format(num_tries))
                time.sleep(0.01)
                num_tries = num_tries + 1
            
        logString("Connected")

    def transmit(self, goal_angles):
        # Convert motor array from the coordinate system used by controls to
        # that used by embedded
        self.goal_angles = ctrlToMcuAngles(goal_angles)
        
        # Send over serial
        sendPacketToMCU(self.ser, vec2bytes(self.goal_angles))
        self.num_transmissions = self.num_transmissions + 1
        
    def receive(self):
        (receive_succeeded, buff) = receivePacketFromMCU(self.ser)
        if(receive_succeeded):
            # If our reception was successful, we update the class variables
            # for the received angles and received IMU data. Otherwise, we
            # just send back the last values we got successfully
            (recvAngles, recvIMUData) = rxDecoder(buff)
            
            angleArray = np.array(recvAngles)
            self.received_angles = angleArray[:, np.newaxis]
            self.received_imu = np.array(recvIMUData).reshape((6, 1))
            self.num_receptions = self.num_receptions + 1
        
        if(self.isROSmode):
            self.publish_sensor_data()
            
    def publish_sensor_data(self):
        ''' IMU Feedback '''
        imu = Imu()
        vec1 = Vector3(-self.received_imu[2][0], self.received_imu[1][0], self.received_imu[0][0])
        imu.angular_velocity = vec1
        vec2 = Vector3(self.received_imu[5][0], self.received_imu[4][0], self.received_imu[3][0])
        imu.linear_acceleration = vec2
        pub.publish(imu)
        
        ''' Motor Feedback '''
        # Convert motor array from the embedded coordinate system to that
        # used by controls
        ctrlAngleArray = mcuToCtrlAngles(self.received_angles)
                
        robotState = RobotState()
        for i in range(12):
            robotState.joint_angles[i] = ctrlAngleArray[i][0]
        
        # Convert motor array from the embedded order and sign convention
        # to that used by controls
        m = getCtrlToMcuAngleMap()
        robotState.joint_angles[0:12] = np.linalg.inv(m).dot(robotState.joint_angles[0:18])[0:12]

        pub2.publish(robotState)
            
    def print_handler(self):
        current_time = time.time()
        if(current_time - self.last_print_time >= 1):
            self.last_print_time = current_time
            print('\n')
            logString("Received: {0}\n".format(self.num_receptions))
            logString("Transmitted: {0}\n".format(self.num_transmissions))
            printAsAngles(self.goal_angles[0:12], self.received_angles[0:12])
            printAsIMUData(self.received_imu)
            
    def communicate(self, goal_angles):
        while not success:
            # This variable is just used to see whether we run into an exception
            success = False
            try:
                self.transmit(goal_angles)
                self.receive()
                self.print_handler()
                success = True
            
            except serial.serialutil.SerialException as e:
                self.ser.close()
                logString("Serial exception {0}. Closed port. Retrying connection".format(e))
                self.connectToEmbedded()

    def trajectory_callback(self, robotGoal):
        # Convert motor array from the order and sign convention used by
        # controls to that used by embedded
        m = getCtrlToMcuAngleMap()
        goalangles[0:18,0] = m.dot(robotGoal.trajectories[0:18])
        self.communicate(goal_angles)
    
if __name__ == "__main__":
    sh = soccer_hardware()
    sh.connect_to_embedded()
    
    if(sh.isROSmode):
        rospy.init_node('soccer_hardware', anonymous=True)
        rospy.Subscriber("robotGoal", RobotGoal, sh.trajectory_callback, queue_size=1)
        pub = rospy.Publisher('soccerbot/imu', Imu, queue_size=1)
        pub2 = rospy.Publisher('soccerbot/robotState', RobotState, queue_size=1)
        rospy.spin() 
    else:
        while(True):
            # Iterate through the static trajectory forever...
            # TODO: If the static trajectories are ever re-generated, we will need
            # TODO: to dot the trajectories with the ctrlToMcuAngleMap to shuffle
            # TODO: them properly
            for i in range(self.trajectory.shape[1]):
                goal_angles = self.trajectory[:, i:i+1]
                sh.communicate(goal_angles)
    
    sys.exit(0)