# encodercomm
Encoder communicator for [PermoCar](https://github.com/Trobolit/PermoCar) project.
In these node is a serial communication from a Arduino containing encoder data reserved. The data is then past on as a ROS messages.

## Subscribe
* serial communication on port ttyACM2

## Publish
* topic wheel_velocity, std_msgs/Float32MultiArray ([0] = left wheel speed, [1] = right wheel speed)
