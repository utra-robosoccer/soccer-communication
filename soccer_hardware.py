#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created June 6 2018

Author: Tyler and Jason
"""

import argparse
import serial
import serial.tools.list_ports
import time
import os
import sys
import numpy as np
import struct

from datetime import datetime
from prettytable import PrettyTable

from transformations import *

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
    
except:
    has_ros = False

def logString(userMsg):
    ''' Prints the desired string to the shell, precedded by the date and time.
    '''
    print(datetime.now().strftime('%H.%M.%S.%f') + " " + userMsg)

def list_ports():
    ports = serial.tools.list_ports.comports()
    msg = ""
    if(len(ports) == 0):
        msg = "Error: No COM ports have been detected"
    else:
        ports = [port.device for port in ports]
        msg = "Available ports are: " + " ".join(ports)
    return msg

def get_script_path():
    return os.path.dirname(os.path.realpath(sys.argv[0]))
    
def parse_args():
    os.chdir(get_script_path())
    logString("Starting PC-side application")
    
    parser = argparse.ArgumentParser(description='Soccer hardware')
    parser.add_argument(
        '-r',
        '--ros',
        help='Subscribes and publishes to ROS nodes (event-based). This is '
                'the flow used on the actual robot. Default: True if you have '
                'ROS installed, otherwise false',
        default=True
    )
    parser.add_argument(
        '--port',
        help='The serial port used for communication. Default: /dev/ttyUSB0',
        default='/dev/ttyACM0'
    )
    
    parser.add_argument(
        '--baud',
        help='Serial port baud rate. Default: 230400',
        default=230400
    )
    
    parser.add_argument(
        '--traj',
        help='The trajectory to be used, if not in ROS mode. Default: '
                'standing.csv',
        default='standing.csv'
    )
    
    parser.add_argument(
        '--step',
        help='Causes goal angles to be sent when enter is pressed, if not '
                'in ROS mode. Default: False',
        default=False
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
    
    arg_str = ""
    for k in args.keys():
        if(k == 'ros' and not has_ros):
            original_arg = args[k]
            args[k] = False
            
            arg_str = arg_str + k + "=" + str(False)
            if(original_arg == True):
                arg_str = arg_str + " (ROS is not installed)"
                
            arg_str = arg_str + ", "
        else:
            arg_str = arg_str + k + "=" + str(args[k]) + ", "

    logString("Starting script with " + arg_str[:len(arg_str) - 2])
    
    return args

class Transmitter:
    def __init__(self, ser):
        self.ser = ser
        self.num_transmissions = 0
        
    def send_packet_to_mcu(self, byteStream):
        ''' Sends bytes to the MCU with the header sequence attached.
        '''
        header = struct.pack('<L', 0xFFFFFFFF)
        id = struct.pack('<I', 0x1234)
        padding = bytes(''.encode())
        footer = struct.pack('<L', 0x00000000)
        
        numBytes = len(byteStream)
        if(numBytes < 80):
            padding = struct.pack('<B', 0x00) * (80 - numBytes)
            
        self.ser.write(header + id + byteStream + padding + footer)
        
    def vec2bytes(self, vec):
        ''' Transforms a numpy vector to a byte array, with entries interpreted as
            32-bit floats.
        '''
        byteArr = bytes(''.encode())
        for element in vec:
            byteArr = byteArr + struct.pack('f', element)
        return byteArr
        
    def transmit(self, goal_angles):
        '''
        Converts the motor array from the coordinate system used by controls to
        that used by embedded and sends it to the MCU over serial
        '''
        goal_angles = ctrlToMcuAngles(goal_angles)
        
        self.send_packet_to_mcu(self.vec2bytes(goal_angles))
        self.num_transmissions = self.num_transmissions + 1
        
class Receiver:
    def __init__(self, ser, ros_is_on):
        self.ser = ser
        self.ros_is_on = ros_is_on
        self.num_receptions = 0
        
    def decode(self, raw):
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
        
    def receive_packet_from_mcu(self):
        '''
        Receives 80 bytes of the MCU provided that there is a valid 4-byte 
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
        
        num_bytes_available = 0
        while(True):
            # First, we wait until we have received some data, or until the timeout
            # has elapsed
            while((num_bytes_available == 0) and (time_curr - time_start < timeout)):
                time.sleep(0.001)
                time_curr = time.time()
                num_bytes_available = self.ser.in_waiting
            
            if((num_bytes_available == 0) and (time_curr - time_start >= timeout)):
                break
            else:
                # If we receive some data, we process it here then go back to
                # waiting for more
                rawData = self.ser.read(num_bytes_available)
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
                num_bytes_available = 0
                            
        return (receive_succeeded, buff)
        
    def publish_sensor_data(self):
        # IMU FEEDBACK
        imu = Imu()
        vec1 = Vector3(-self.received_imu[2][0], self.received_imu[1][0], self.received_imu[0][0])
        imu.angular_velocity = vec1
        vec2 = Vector3(self.received_imu[5][0], self.received_imu[4][0], self.received_imu[3][0])
        imu.linear_acceleration = vec2
        pub.publish(imu)
        
        # MOTOR FEEDBACK
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
        
    def receive(self):
        (receive_succeeded, buff) = self.receive_packet_from_mcu()
        if(receive_succeeded):
            # If our reception was successful, we update the class variables
            # for the received angles and received IMU data. Otherwise, we
            # just send back the last values we got successfully
            (recvAngles, recvIMUData) = self.decode(buff)
            
            angleArray = np.array(recvAngles)
            self.received_angles = angleArray[:, np.newaxis]
            self.received_imu = np.array(recvIMUData).reshape((6, 1))
            self.num_receptions = self.num_receptions + 1
        
        if(self.ros_is_on):
            self.publish_sensor_data()
        
class Comm:
    def __init__(self):
        self.first = True
        self.last_print_time = time.time()
        
    def start_up(self, ser, ros_is_on, traj, step_is_on):
        self.ser = ser
        self.ros_is_on = ros_is_on
        self.step_is_on = step_is_on
        self.use_trajectory = False
        
        self.tx = Transmitter(ser)
        self.rx = Receiver(ser, ros_is_on)
        
        if(self.ros_is_on):
            if(self.first):
                rospy.init_node('soccer_hardware', anonymous=True)
                rospy.Subscriber("robotGoal", RobotGoal, sh.trajectory_callback, queue_size=1)
                pub = rospy.Publisher('soccerbot/imu', Imu, queue_size=1)
                pub2 = rospy.Publisher('soccerbot/robotState', RobotState, queue_size=1)
                self.first = False
        else:
            trajectories_dir = os.path.join("trajectories", traj)
            try:
                self.trajectory = np.loadtxt(
                    open(trajectories_dir, "rb"),
                    delimiter=",",
                    skiprows=0
                )
                
                logString("Opened trajectory {0}".format(traj))
                self.use_trajectory = True
            except IOError as err:
                logString("Error: Could not open trajectory: {0}".format(err))
                logString("(Is your shell running in the soccer-communication directory?)")
                logString("Standing pose will be sent instead...")

    def print_angles(self, sent, received):
        ''' Prints out 2 numpy vectors side-by-side, where the first vector entry
            is interpreted as belonging to motor 1, the seconds to motor 2, etc.
        '''
        assert sent.shape[0] == received.shape[0]
        t = PrettyTable(['Motor Number', 'Sent', 'Received'])
        
        for i in range(sent.shape[0]):
            t.add_row([str(i + 1), round(sent[i][0], 4), round(received[i][0], 2)])
        
        print(t)
    
    def print_imu(self, received):
        ''' Prints out a numpy vector interpreted as data from the IMU, in the
            order X-gyro, Y-gyro, Z-gyro, X-accel, Y-accel, Z-accel.
        '''
        
        t = PrettyTable(['', 'Gyro (deg/s)', 'Accel (m/s^2)'])
        
        t.add_row(["X", round(received[0][0], 2), round(received[3][0], 2)])
        t.add_row(["Y", round(received[1][0], 2), round(received[4][0], 2)])
        t.add_row(["Z", round(received[2][0], 2), round(received[5][0], 2)])
        
        print(t)

    def print_handler(self, goal_angles):
        current_time = time.time()
        if(current_time - self.last_print_time >= 1):
            self.last_print_time = current_time
            print('\n')
            logString("Received: {0}".format(self.rx.num_receptions))
            logString("Transmitted: {0}\n".format(self.tx.num_transmissions))
            self.print_angles(goal_angles[0:12], self.rx.received_angles[0:12])
            self.print_imu(self.rx.received_imu)
    
    def communicate(self, goal_angles):
        self.tx.transmit(goal_angles)
        self.rx.receive()
        self.print_handler(goal_angles)
    
    def trajectory_callback(self, robotGoal):
        '''
        Used by ROS. Converts the motor array from the order and sign convention
        used by controls to that used by embedded
        '''
        m = getCtrlToMcuAngleMap()
        goalangles[0:18,0] = m.dot(robotGoal.trajectories[0:18])
        self.communicate(goal_angles)
    
    def begin_event_loop(self):
        if(self.ros_is_on):
            rospy.spin() 
        else:
            while(True):
                if(self.use_trajectory):
                    # Iterate through the static trajectory forever
                    # TODO: If the static trajectories are ever re-generated, we will need
                    # TODO: to dot the trajectories with the ctrlToMcuAngleMap to shuffle
                    # TODO: them properly
                    for i in range(self.trajectory.shape[1]):
                        if(self.step_is_on):
                            wait = input('Press enter to send next pose')
                        
                        goal_angles = self.trajectory[:, i:i+1]
                        self.communicate(goal_angles)
                else:
                     # Send standing pose
                    if(self.step_is_on):
                        wait = input('Press enter to send next pose')

                    self.communicate(np.zeros((18,1)))
        
def main():
    args = parse_args()
    
    ros_is_on = args['ros']
    port = 'COM3' #args['port']
    baud = args['baud']
    traj = args['traj']
    step_is_on = args['step']
    
    logString(list_ports())
    
    logString("Attempting connection to embedded")
    logString("\tPort: " + port)
    logString("\tBaud rate: " + str(baud))
    
    num_tries = 0
    comm = Comm()
    while(True):
        try:
            with serial.Serial(port, baud, timeout=0) as ser:
                logString("Connected")
                comm.start_up(ser, ros_is_on, traj, step_is_on)
                comm.begin_event_loop()
        
        except serial.serialutil.SerialException as e:
            if(num_tries % 100 == 0):
                if(str(e).find("FileNotFoundError")):
                    logString("Port not found. Retrying...(attempt {0})".format(num_tries))
                else:
                    logString("Serial exception. Retrying...(attempt {0})".format(num_tries))
            
            time.sleep(0.01)
            num_tries = num_tries + 1

if __name__ == "__main__":
    try:
        main()
        sys.exit(0)
    except KeyboardInterrupt as e:
        print("Interrupted: {0}".format(e))
        sys.exit(1)