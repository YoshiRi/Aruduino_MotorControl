///////////////////////////////////////////
//  Yoshi Ri @ Univ Tokyo
//  8 April 2018
//////////////////////////////////////////

// for servo motor control
#include <Servo.h>
#include <Encoder.h>
// ROS communication
//#include <ArduinoHardware.h>
//#include <ArduinoTcpHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

/*--  PID function --*/
// for ATmega2560 it must be AVR
#include "inttypes.h"
#include <ArduPID.h>
//#include <PID_def.h>
//#include <PID_IC.h>
//#include <PID.h>
//#include <PID_BC.h>
/*--  PID function --*/

ros::NodeHandle  nh;

#define COUNTS_PER_ROTATION 17952 // Not exact
#define WHEEL_DIAMETER 0.11 // Not exact too
#define METERS_PER_COUNT 3.14 * WHEEL_DIAMETER / COUNTS_PER_ROTATION
#define SAMPLING 5 // sampling 5ms
#define INPUTLIMIT 0.05 // m/s Limitation of reference

// This depends your environment
Encoder myEnc(2, 3);

// Servo ........ 
Servo myservoA;

// initial values
int SteerAngle = 90;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=400, Ki=0, Kd=0;
double N = 100; //derivative filter constant D(s)=s/(1+s/N)
uint32_t T = SAMPLING;
// Constructor
PID_IC myPID( &Output, Kp, Ki, Kd, N, T);

// DC motor set pin number this depends on your hardware
const int 
PWM_B   = 11,
DIR_B   = 13,
BRAKE_B = 8,
SNS_B   = A1;


// odd global variables to save pose, time
long oldPosition  = 0;
long newPosition, dest = 0;
long prevTime, currTime, cmdTime;
long outputPWM = 0;

// Position for publishing data
    // Initialize pose
double x = 0.0;
double y = 0.0;
double th = 0.0;
const double leng = 0.275;
double dt,vx,vy,vth,st_ang;


// Subscriber call back function
void cmd_velCb( const geometry_msgs::Twist& CVel){
  Setpoint =  (double) CVel.linear.x;
  SteerAngle =  90 - (int) (CVel.angular.z * 180 / 3.14);
  cmdTime = millis();
  myPID.Reset(); // Reset integer
}

ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel", &cmd_velCb );

// Publisher
geometry_msgs::Twist CurrPos;
ros::Publisher Current_Pos("current_vel", &CurrPos);

String inString = "";

void setup() {
  // Servo ......
  myservoA.attach(5);  //  Servo motor pin 5 
  myservoA.write(SteerAngle);

  
  // DC motor .......
  // Configure the B output
  pinMode(BRAKE_B, OUTPUT);  // Brake pin on channel B
  pinMode(DIR_B, OUTPUT);    // Direction pin on channel B
  cmdTime = millis();
  prevTime = millis();
  Setpoint = 0.0f;
  oldPosition = myEnc.read();
  //turn the PID on
  myPID.SetSaturation(-5,5);
  myPID.Reset();
  
  delay(1000);
  nh.getHardware()->setBaud(115200); //set baudrate before init
  nh.initNode();
  nh.subscribe(sub_cmd_vel);
  nh.advertise(Current_Pos);
}


void loop() {
  nh.spinOnce();
  delay(1);
  
  // get stearing angle
  st_ang = double(90 - myservoA.read()) * 3.141492/180.0;
  
  // if signal is stopped then stop
  if (millis() - cmdTime > 3000)  {
    SteerAngle = 90;
    Setpoint = (double) 0;
  }
  

  // Get current pose
  newPosition = myEnc.read();
  currTime = millis();
  dt = double(currTime - prevTime) / 1000.0;
  Input = -(double)(newPosition - oldPosition) / dt; // Velocity in counts/sec
  Input = Input * METERS_PER_COUNT; // Velocity in meters/sec
  if (Input > INPUTLIMIT) Input = INPUTLIMIT; // HotFix : Limit in case of buffer overflow. Bug
  prevTime = currTime;
  oldPosition = newPosition;

  // calc odometry
  vx = cos(th)*Input;
  vy = sin(th)*Input;
  vth = tan(st_ang)/leng;

  x += vx*dt;
  y += vy*dt;
  th += vth*dt;
  
  //Publish data to calculate odometry
  CurrPos.linear.x = x;
  CurrPos.linear.y = y;
  CurrPos.linear.z = 0;
  CurrPos.angular.z = th;
  Current_Pos.publish(&CurrPos);
  
  
  // Get error from setpoint and current value
  double error = Setpoint - Input;
  if (myPID.AutoCompute(error))  {
    // convert to the pwm value 
    outputPWM = (long)(Output*255.0/5.0);
  }
  
  // move steering angle
  myservoA.write(SteerAngle);
  
  // move DC motor
  if (outputPWM > 0)  {
    digitalWrite(DIR_B, HIGH);   // setting direction to HIGH the motor will spin forward
  } else  {
    digitalWrite(DIR_B, LOW);
  }
  
  analogWrite(PWM_B, abs(outputPWM));     // Set the speed of the motor, 255 is the maximum value
}
