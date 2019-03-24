# Receiver.py

import serial
import numpy as np
import struct
from threading import Thread, Event
from transformations import *

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
    pass

class Receiver:
    def __init__(self, ser, dryrun):
        self._ser = ser
        self._dryrun = dryrun
        
    def _get_fake_packet(self):
        header = struct.pack('<L', 0xFFFFFFFF)
        id = struct.pack('<I', 0x1234)
        payload = struct.pack('<B', 0x00)*80
        footer = struct.pack('<L', 0x00000000)
        packet = header + id + payload + footer
        return packet
        
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
        
        if self._dryrun:
            return (True, self._get_fake_packet())
        
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
                num_bytes_available = self._ser.in_waiting
            
            if((num_bytes_available == 0) and (time_curr - time_start >= timeout)):
                break
            else:
                # If we receive some data, we process it here then go back to
                # waiting for more
                rawData = self._ser.read(num_bytes_available)
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
                if receive_succeeded:
                    break
        return (receive_succeeded, buff)


class Rx(Thread):
    def __init__(self, group=None, target=None, name=None, args=(), **kwargs):
        super(Rx, self).__init__(group=group, target=target, name=name)
        self.args = args
        self.kwargs = kwargs
        self._name = name
        self._stop_event = Event()
        self._num_rx = 0
        self._dryrun = kwargs['dryrun']
        self._receiver = Receiver(kwargs['ser'], self._dryrun)
        self._imu_payload = np.ndarray(shape=(6,1))
        self._angles_payload = np.ndarray(shape=(12,1))

    def stop(self):
        '''
        Causes the thread to exit after the next receive event (whether a
        success or a failure)
        '''
        self._stop_event.set()

    def _stopped(self):
        return self._stop_event.is_set()
        
    def get_num_rx(self):
        '''
        Returns the number of successful receptions
        '''
        return self._num_rx
        
    def bind(self, callback):
        '''
        Attaches a function to be called after a successful reception
        '''
        self._callback = callback

    def run(self):
        '''
        Reads packets from the microcontroller and sends the data up to the
        application through a callback function
        '''
        while(1):
            if self._stopped():
                print("Stopping Rx thread ({0})...".format(self._name))
                return
            else:
                (receive_succeeded, buff) = self._receiver.receive_packet_from_mcu()
                if receive_succeeded:
                    # If our reception was successful, we update the class variables
                    # for the received angles and received IMU data. Otherwise, we
                    # just send back the last values we got successfully
                    (recvAngles, recvIMUData) = self._receiver.decode(buff)
                    
                    angleArray = np.array(recvAngles)
                    received_angles = angleArray[:, np.newaxis]
                    received_imu = np.array(recvIMUData).reshape((6, 1))
                    self._num_rx = self._num_rx + 1
                    self._callback(received_angles, received_imu)


def test_callback(received_angles, received_imu):
    print("\tGot angle data {0}".format(received_angles.shape))
    print("\tGot IMU data {0}".format(received_imu.shape))
    

if __name__ == "__main__":
    rx_thread = Rx(name="rx_th", ser="", dryrun=True)
    rx_thread.bind(test_callback)
    rx_thread.start()
    while rx_thread.get_num_rx() < 20s:
        pass
    rx_thread.stop()
    rx_thread.join()
    print("Stopping main")