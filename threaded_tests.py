import time
import os
import numpy as np
from threading import Thread, Event
from queue import Queue

class Tx(Thread):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs=None):
        super(Tx, self).__init__(group=group, target=target, name=name)
        self.args = args
        self.kwargs = kwargs
        self.print_queue = Queue(10)
        self._stop_event = Event()
        self._time = time.time()

    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()
        
    def send(self, str_to_print):
        self.print_queue.put(str_to_print)

    def run(self):
        while(1):
            if self.stopped():
                print("Stopping Tx...")
                return
            if not self.print_queue.empty():
                my_str = self.print_queue.get()
                print(my_str)
            elif time.time() - self._time > 1:
                self._time = time.time()
                print("Tx: " + str(self._time))


class Rx(Thread):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs=None):
        super(Rx, self).__init__(group=group, target=target, name=name)
        self.args = args
        self.kwargs = kwargs
        self.print_queue = Queue(10)
        self._stop_event = Event()
        self._time = time.time()
        self._payload = np.ndarray(shape=(6,1))

    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()
        
    def bind(self, callback):
        self._callback = callback

    def run(self):
        while(1):
            if self.stopped():
                print("Stopping Rx...")
                return
            elif time.time() - self._time > 0.33:
                self._time = time.time()
                self._callback(self._payload)
                print("Rx: " + str(self._time))


def callback(imu_data):
    print("IMU data: {0}".format(imu_data))


if __name__ == "__main__":
    tx_thread = Tx()
    tx_thread.start()
    
    rx_thread = Rx()
    rx_thread.bind(callback)
    rx_thread.start()
    
    i = 0
    while True:
        time.sleep(0.10)
        i = i + 1
        tx_thread.send(str(i))
        if i == 20:
            break
    
    tx_thread.stop()
    tx_thread.join()
    
    rx_thread.stop()
    rx_thread.join()
    
    print("Stopping main")