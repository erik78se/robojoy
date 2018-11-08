# Description
The code in this repo is a complete catkin workspace. IS has all nessesary code and docs to get an example robot car to run with the Robot Operating System - ROS. The robot will be controlled via a USB joystick. Its part of a workshop material to help friends get started with ROS and to make awesome things with robots.

The repo contains basically two things:
 - A ROS package named "robojoy" - which is the name of our robot. It is in this case a simple "launch" file that starts our various parts of the robot (ROS nodes).
 - Ardunio code - that translates "Twist" messages on the serial port into PWM for 2 motors. (ROS subscriber pattern) 

# Bill of Material
 - RPi2 (or 3) with an SD card and WiFi network.
 - A motor control HAT for the RPi. A-Star 32U4 RC LV/SV RPi Bridge	Put on top of the RPI (HAT), has microcontroller + everyting needed for micro-electronics	Interfaces 5V, has h-bridges for motors and AD for analog sensors
 - A USB joystick. We will use this to control robot (forward, backward, left & right).


# Install dependencies with Ansible
 - Add your ssh keys to your users to use for Ansible on all platforms and add sudo rules if needed.
 - Edit inventory.ini to reflect your environment. You can have as many hosts as you like under [ros].
 - Run `ansible-playbook -i inventory.ini site.yml`. This will add all dependencies and is potentially very slow (~30 min is not unusual). It will also add a user called 'robot' with ssh keys added.
 - It checks out robojoy git repo under /home/robot/catkin_ws.

# NOTE!
At this point, we dont cover more on the ROS side. We assume you have:
 - A working installation of ROS on your RPi.
 - You can communicate with the Arduino through the serial port on the RPi. 
 - You know what device the joystick has when you plug it into the USB port. Usually /dev/input/js1.
 - You have rosserial_arduino, rosserial_msgs, rosserial_python packages installed. (https://github.com/ros-drivers/rosserial) 
 - You have ROS joystick packages installed: joy & teleop_twist_joy

# Get the catkin workspace 
Cloning this repo into catkin_ws
```
$ cd ~
$ git clone https://github.com/erik78se/robojoy catkin_ws
$ git submodule update --init --recursive
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
The code should now be automatically uploaded to your arduino via the serial interface. This may or may not work for your setup. Dont worry. You can load the arduino code by any means favoured by you as long as the serial communication works between the RPi and the arduino at later stage.

# Checkpoint - Pause and reflect before moving on.
Before moving on, you should now have:
 - Installed ROS on your Rpi
 - Verified your joystick works and have the device name (like: /dev/input/js1)
 - Verified the serial communication between the RPi and the arduino device (like /dev/ttyAMA0).
 - Successfully uploaded the ardunio code to your arduino/MCU

# Testing the joystick (manually) with ROS

### Start master
```
$ roscore -v
```
#### Start joystick node
Responsible for communication with the joystick
```
$ rosparam set /joy_node/dev /dev/input/js1
$ rosrun joy joy_node
```

### Start teleop_twist_joy node
Responsible for converting joystick commands to 'Twist' messages used by ROS and put them on the /cmd_vel topic.
```
$ rosrun teleop_twist_joy teleop_node
```
### Test to see twist messaged on /cmd_vel topic
```
$ rostopic echo /cmd_vel
```
Press buttons on your joystick (you might need to hold and press an "enable" button on your joystick to see anything happen with you turn the sticks in the joystick.

### Start the rosserial_python node
This node is responsible to send the communication (Twist messages) to the ardunio
```
$ rosrun rosserial_python serial_node.py /dev/ttyAMA0
```

### The ROS way. Use the launchfile
All of the above manual steps can be done via ".launch files". 

# Ardunio extra (not part of this excersise): 
To enable programming to an Arduino-mini-pro from Serial
> Wire connect: arduino(DRT) -> RPi(GPIO #17)
``` 
$ apt-get install arduino-mk
```
Then make upload might work
