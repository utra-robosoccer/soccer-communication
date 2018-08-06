# soccer-communication
The PC side communication node for communicating with the microcontroller

### Required Programs on the PC computer before running
```
pip3 install pyserial
pip install numpy --user
pip install prettytable --user
```
### To Run the program on ROS
```
roslaunch soccer_hardware soccer_hardware.py
```

### To Run the program with a trajectory. You need to press the space button to send the next trajectory in a file
```
python soccer_hardware <path/to/trajectory>
```
