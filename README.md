# ins_gamepad_interaction
Virtual interaction in gazebo simulator with an INS (inertial navigation system) gamepad.

## Demo test
Get gamepad running and magnetometer calibrated. Get the coordinator xbee and arduino connected with computer.
```
roscore
```
```
rosrun rosserial_python serial_node.py /dev/ttyUSB0
```
The topic "ins_gamepad" should be publishing now.
```
roslaunch ins_gamepad_interaction drill_3D_orientation.launch
```


(add pictures for gamepad, drill, bowling, beercan)

