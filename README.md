# ins_gamepad_interaction
Virtual interaction in gazebo simulator with an INS (inertial navigation system) gamepad.
Only the part concerned with ROS are included here.

## Demo test
Get gamepad running and magnetometer calibrated. Get the coordinator xbee and arduino connected with computer.
```
roscore
```
```
rosrun rosserial_python serial_node.py /dev/ttyUSB0
```
The topic "ins_gamepad" should be publishing now.
Test 1: A drill rotates with orentation info from gamepad
```
roslaunch ins_gamepad_interaction drill_3D_orientation.launch
```
Test 2: A beer moves on the ground with 2D position info from gamepad
```
roslaunch ins_gamepad_interaction beer_planar_position.launch
```

(add pictures for gamepad, drill, beercan)

