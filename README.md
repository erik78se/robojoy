# Setup
* ROS version: jade
* rosserial_python

# The robot consists of a RPi2 + Arduino Mini Pro 3.3v
# A joystick sends Twist messages to the Ardunio that controls the 2 motors (left & right).

# Enable programming the Arduino-mini-pro from Serial
# Arduino(DRT) -> GPI17(Rpi2)
apt-get install arduino-mk

# Create a package "robojoy" with dependencies ("robojoy" is what you git clone from this repo)
catkin_create_pkg robojoy std_msgs rospy roscpp joy teleop_twist_joy rosserial_python


# Load arduino code onto the robot via serial interface

cd catkin_ws/arduino/
make upload

cd catkin_ws/
source ./devel/setup.sh

# On master
roscore -v

# On joystick node
rosrun joy joy_node

# Potentially change joystick device: 

rosparam set /joy_node/dev /dev/input/js1


# On joystick node
rosrun teleop_twist_joy teleop_node

# Test to see twist messaged on /cmd_vel topic
rostopic echo /cmd_vel

# On the arduino serial connected node
rosrun rosserial_python serial_node.py /dev/ttyAMA0

# Alternative, use the launchfile
