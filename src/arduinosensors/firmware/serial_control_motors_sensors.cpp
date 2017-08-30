/* serial_control_2_motors_pwm.ino:
 *
 *  This code implements a motor control for a simple
 *  ROS robot, connected to a arduino via serial interface, and 2 DC motors(PWM).
 *  The wheels is ideally tracks.
 *
 *  To test it, you could use a USB joystick.
 *
 * Get 3 ROS packages/nodes to work first:
 * (joy/joy_node)
 * (rosserial_python/serial_node.py)
 * (teleop_twist_joy/teleop_node)
 *
 *  Test that Twist messages appear on the /cmd_vel topic
 *
 *  The principle is that
 *
 *  I)   <Twist> messages are published on /cmd_vel
 *  II)  The arduino subscribes to /cmd_vel
 *  III) Arduino translates the Twist message into
 *  IV)  PWM speed for left and right motors.
 *
 *  Make sure that the arduino code is compiled with the same version
 *  of "rosserial_python", or you might get protocol errors like:
 *  [ERROR] [WallTime: 1463602684.416179] Mismatched protocol version in packet:
 *  lost sync or rosserial_python is from different ros release than the rosserial client
 *
 *  Below is a "launch" file for ROS that shows the stack you need to get it running.
 *
 *  <launch>
 *  <node name="joy1" pkg="joy" type="joy_node" />
 *  <node name="teleoptwistjoy1" pkg="teleop_twist_joy" type="teleop_node" />
 *  <node name="serialcommjoy1" pkg="rosserial_python" type="serial_node.py" args="/dev/ttyAMA0"/>
 *  </launch>
 *
 *
*/
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

// SERIAL COMMUNICATION
#define ROS_SERIAL_BAUD_RATE 57600

// MOTORS
#define E1 9   // Enable Pin for motor 1
#define E2 10    // Enable Pin for motor 2
#define FRONT_IR 12  //Collission front pin
#define BACK_IR  4   //Collission back pin


#define FORWARD1 digitalWrite(12, HIGH);
#define REVERSE1 digitalWrite(12, LOW);
#define FORWARD2 PORTE |= 0x04;
#define REVERSE2 PORTE &= 0xFB;
ros::NodeHandle nh;

geometry_msgs::Twist twist_msg;

// The PWM values we are going to use
double lspeed = 0;
double rspeed = 0;

void MotorSpeeds(float left, float right) {

  // Raise values -0.5 - +0.5 with a factor of 255
  // So that we can have useful PWM values (0-255)
  // Perhaps, we could do a map() here instead.
  
  lspeed = left*255;
  rspeed = right*255;
  
  analogWrite(E1, abs(lspeed));  // Set left motor speed
  analogWrite(E2, abs(rspeed));  // Set right motor speed


  // Motor direction is determined based on the sign of the left,right values
  // Set direction of the motors based on the sign of the speed
  if ( left >= 0 ) {
    FORWARD1;
  } else {
    REVERSE1;
  }
  if ( right >= 0 ) {
    FORWARD2;
  } else {
     REVERSE2;
  }

}

////////////////////////////////////////////////////////////
// Motor drive
// cmd_x  : linear x velocity (forward velocity) m/s
// cmd_th : angular z velocity (rotation of heading) rad/s
void diff_drive(float cmd_x, float cmd_th) {
  
  char log_msg[200];
  // dtostrf(floatVar, minStringWidthIncDecimalPoint, numVarsAfterDecimal, charBuf);
  dtostrf(cmd_x*255, 1, 4, log_msg);
  nh.loginfo(log_msg);

  // Calculate speed to send to the motors.
  MotorSpeeds( cmd_x-cmd_th, cmd_x+cmd_th );
}

// The callback function
void cmd_vel_callback( const geometry_msgs::Twist& twist_msg) {
  diff_drive(twist_msg.linear.x, twist_msg.angular.z);
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel_topic("cmd_vel", &cmd_vel_callback);

// Collission message
std_msgs::String str_msg;
char fbump[7] = "f_bump";
char bbump[7] = "b_bump";
ros::Publisher front_ir("front_ir", &str_msg);
ros::Publisher back_ir("back_ir", &str_msg);


// Setup the pin numbers.
void setupPins() {
  delay(10000); // Uggly fix to avoid bootloop on RPi with the HAT
  pinMode(E1, OUTPUT);
  pinMode(E2, OUTPUT);
  pinMode(12, OUTPUT); // Set 12 to output (motor1)
  DDRE |= 0x04; // Set PE2 to output (motor2)
}

void setup() {
  nh.getHardware()->setBaud(ROS_SERIAL_BAUD_RATE);
  nh.initNode();
  nh.subscribe(cmd_vel_topic);
  nh.advertise(front_ir);
  nh.advertise(back_ir);
  nh.loginfo("Serial setup complete");
  
  setupPins();
  nh.loginfo("Pins setup complete");
}


int frontIRState;
int backIRState;

void loop() {
 
  // Handle collissions by reading IR sensors.
  frontIRState = digitalRead(FRONT_IR);
  backIRState = digitalRead(BACK_IR);

  if ( frontIRState == 0 ) {
      str_msg.data = fbump;
      front_ir.publish( &str_msg );
  }

  if ( backIRState == 0 ) {
      str_msg.data = bbump;
      back_ir.publish( &str_msg );
  }

  nh.spinOnce();
  delay(1);  
}
