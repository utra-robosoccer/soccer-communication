# soccer-communication
The PC side communication node for communicating with the microcontroller

### Required programs on the PC computer before running
```
pip3 install pyserial
pip install numpy --user
pip install prettytable --user
```
### To rRun the program on ROS
```
roslaunch soccer_hardware soccer_hardware.py
```

### Command-line arguments
There are currently 4 supported command-line arguments:
- `--ros` whether or not to import ROS dependencies and execute the logic used on the actual robot, or omit ROS dependencies and simply loop through a local static trajectory file
- `--port` the name of the virtual COM port associated with the microcontroller
- `--baud` the baud rate of the virtual COM port
- `--traj` name of the trajectory file to load at startup. Must be the name of the file, including ".csv" (note: not the path. The path to the trajectories folder is automatically prepended)

Example:
```
python soccer_hardware.py --ros=False --port=COM7 --baud=1000000 --traj=getupfront.csv
```
