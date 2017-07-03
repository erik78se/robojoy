# Description
The code in this repo is a complete catkin workspace. IS has all nessesary code and docs to get an example robot car to run with the Robot Operating System - ROS. The robot will be controlled via a USB joystick. Its part of a workshop material to help friends get started with ROS and to make awesome things with robots.

The repo contains basically two things:
 - A ROS package named "robojoy" - which is the name of our robot. It is in this case a simple "launch" file that starts our various parts of the robot (ROS nodes).
 - Ardunio code - that translates "Twist" messages on the serial port into PWM for 2 motors. (ROS subscriber pattern) 

# Bill of Material
 - RPi2 (or 3) with an SD card and WiFi network.
 - A motor control HAT for the RPi. A-Star 32U4 RC LV/SV RPi Bridge	Put on top of the RPI (HAT), has microcontroller + everyting needed for micro-electronics	Interfaces 5V, has h-bridges for motors and AD for analog sensors
 - A USB joystick. We will use this to control robot (forward, backward, left & right).

# NOTE!
At this point, we dont cover more on the ROS side. We assume you have:
 - A working installation of ROS on your RPi.
 - You can communicate with the Arduino through the serial port on the RPi. 
 - You know what device the joystick has when you plug it into the USB port. Usually /dev/input/js1.

# Get the catkin workspace 
Cloning this repo into catkin_ws
```
cd ~
$ git clone https://github.com/erik78se/robojoy catkin_ws
```
For your information, the workspace you just coloned contains a ROS package and come adruino code. The ROS package was created like this if you like to try on your own:
> catkin_create_pkg robojoy std_msgs rospy roscpp joy teleop_twist_joy rosserial_python

# Build the code to see that is works
```
$ cd catkin_ws
$ catkin_make
```
# Load arduino code onto the robot via serial interface
First, make sure we use the libraries and code from our own workspace. Otherwise, it will use the ROS and OS installation libraries. The following will overload this.
```
$ cd ~
$ source catkin_ws/devel/setup.sh
$ cd catkin_ws/arduino/
$ make upload
```

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

# Enable programming the Arduino-mini-pro from Serial
# Arduino(DRT) -> GPI17(Rpi2)
apt-get install arduino-mk
