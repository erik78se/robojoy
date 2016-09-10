/* Same as: serial_control_2_motors_pwm.ino

* But added front and back IR sensor: IR-sensor-SG035-SZ
  to be able to detect collission

*/
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

// SERIAL COMMUNICATION
#define ROS_SERIAL_BAUD_RATE 57600

// MOTORS
#define E1 10   // Enable Pin for motor 1
#define E2 6    // Enable Pin for motor 2
#define I1 3    // Control pin 1 for motor 1
#define I2 5    // Control pin 2 for motor 1
#define I3 9    // Control pin 1 for motor 2
#define I4 11   // Control pin 2 for motor 2
#define FRONT_IR 12  //Collission front pin
#define BACK_IR  4   //Collission back pin


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
  
  analogWrite(E1, lspeed);  // Set left motor speed
  analogWrite(E2, rspeed);  // Set right motor speed


  // Motor direction is determined based on the sign of the left,right values
  // Set direction of the motors based on the sign of the speed
  if ( left >= 0 ) {
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);
  } else {
    digitalWrite(I1, LOW);
    digitalWrite(I2, HIGH);
  }
  if ( right >= 0 ) {
    digitalWrite(I3, HIGH);
    digitalWrite(I4, LOW);
  } else {
    digitalWrite(I3, LOW);
    digitalWrite(I4, HIGH);
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
  pinMode(E1, OUTPUT);
  pinMode(E2, OUTPUT);
  pinMode(I1, OUTPUT);
  pinMode(I2, OUTPUT);
  pinMode(I3, OUTPUT);
  pinMode(I4, OUTPUT);
  pinMode(FRONT_IR, INPUT);
  pinMode(BACK_IR, INPUT);
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