# CrazyS
CrazyS is an extension of the ROS package RotorS, aimed to modeling, developing and integrating the Crazyflie 2.0

This README will explain what has been done to update and test CrazyS

This version includes an implementation if the Bitcraze firmware PID controller so the Crazyflie may be simulated in a fashion similar to how one could control it with the Crazy Radio and Crazyflie ROS stack https://github.com/whoenig/crazyflie_ros.

Using the Crazyflie ROS stack the drone can be controlled with roll/pitch/yaw commands and a similar control method is made available in the simulator. From the firmware a couple of controller files are added to have the correct implementation of the PID contorller on the Crazyflie.

Added the following files from: https://github.com/bitcraze/crazyflie-firmware 

### cpp
filter.c(pp)

num.c(pp)

pid.c(pp)

### h

filter.h

num.h

pid.h

stabilizer_types.h

## Description
The ROS controller code is just repurposed from Gsilano and are duplicated files with `_v2` added to the name. The files are:

### `crazyflie_rpy_controller_node_v2.cpp/h`
The controller node. Sets up subscriber and publisher and callback function

### `crazyflie_rpy_controller_v2.cpp/h`
Interface between the ros node and the controller. Handles propeller velocity calculations and updating of sensor data to be used by the PID controllers. The controller uses the roll/pitch/yaw values from the odometry sensor and not the sensorfusion state estimation. 

### `crazyflie_onboard_controller_v2.cpp/h`
Interface link between the pid firmware controllers and the ROS node. Calls the pid updates and calculations


## Testing / launch

The launch file will spawn the Crazyflie model and add the rpythrust controller descirbed above. The control signals is handled by a joystick. (Configure to XBOX controller). Left stick = thrust, right stick = roll/pitch, left and right trigger = yaw.

Launch file:

`crazyflie_onboard_v2.launch`


