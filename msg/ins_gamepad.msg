# msg type for ins gamepad
# will be used with both arduino and ros node
# use float32 because float in arduino is also 4 bytes


# quaternion for 3D orientation
# result of sensor fusion and kalman filter
float32 q0_klm
float32 q1_klm
float32 q2_klm
float32 q3_klm
# 2D linear velocity on the horizontal plane
# the velocity is in the sensor frame
float32 vx
float32 vy
# green button status, red is for calibration of the gamepad
bool button_g


