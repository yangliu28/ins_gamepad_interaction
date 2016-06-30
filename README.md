# ins_gamepad_interaction
Virtual interaction in gazebo simulator with an INS (inertial navigation system) gamepad. This is for the project 1 of Algorithm Robotics of Spring2016, only the part concerned with ROS is here, Arduino programs running on gamepad and remote are not included.

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

Check the videos here:

[3D orientation of a drill](https://youtu.be/6nd3b8hlRUQ)

[planar position of a beer](https://youtu.be/hUOOPmMk8YI)
