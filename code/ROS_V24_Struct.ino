#include <ros.h> //required for ROS
#include <ros/time.h> //required for ROS odom
#include <tf/tf.h> //required for ROS odom
#include <tf/transform_broadcaster.h> //required for ROS odom
#include "SimpleRSLK.h" //contains RSLK functions
#include <geometry_msgs/Twist.h>//Twist used for ROS incoming velocity messages
#include <std_msgs/Int8MultiArray.h>//Monitor bumper status
#include <std_msgs/Float32MultiArray.h>//Monitor motor status


#define WHEEL_SEPERATION 0.07 // Distance between wheels in meters 
#define WHEEL_RAD 0.035 //more accurate 0.03475 /* Radius of RSLK wheels in meters 
#define CNT_PER_REVOLUTION 360 // Number of encoder (rising) pulses per wheel revolution 

#define FORWARD 1 //Motor driving forwards, used for identification as well as velocity & error calcuations
#define BACKWARD -1 //Motor driving backwards, used for identification as well as velocity & error calcuations
#define MAX_MOTOR_PWM 240 //Motor expects 8 bit or [0,255] value, recommended to set slightly below true 255 max
#define PROPORTIONAL_CORRECTION 60 //This value is multiplied by the error to determine the size of the increment when adjusting PWM
#define MULT_INV_ONE_MILL 0.000001 //Multiplicative inverse of 1 million, used to convert micro seconds to seconds

struct location{
  float x = 0.0; //location along x relative to fixed world frame
  float y = 0.0; //location along y relative to fixed world frame
  float theta = 0.0; //rotation along z relative to fixed world frame
};

location simulated; //Location based on incomming linear & angular velocity messages
location calculated; //Locattion based on wheel encoder measurments

struct motor{
  int8_t dir = FORWARD; //Direction wheel will propel RSLK 
  float currentVelocity = 0.0; //Current measured linear velocity in m/s
  float desiredVelocity = 0.0; //Linear velocity in m/s required to satisfy incoming linear&angular velocity messages
  uint8_t PWM = 0; //Motor PWM value in range 0-255 inclusive aka [0,255]
};

motor leftMotor; //declare leftMotor
motor rightMotor; //declare rightMotor

/* Function Prototypes*/
float getDistanceTraveled(int side); //returns distance in meters since last called
float getTimeSinceLastUpdate(void);  //returns time in seconds since last called
void updateVelocity(float rightDistance, float leftDistance); //updates global left and right wheel velocity in m/s
void updateLocationSim(void); //updates simulated perfect location transform
void updateLocationEncoder(float leftDist, float rightDist); //updates encoder measured location transform
void timeInit(void); //updates timer variables to prevent unexpected results
void messageCb( const geometry_msgs::Twist& msg); //monitors incomming twist messages to be used for velocity control
void rightMotorControl(float linear, float angular); //adjusts right motor pwm to drive at desired linear & angular velocity
void leftMotorControl(float linear, float angular); //adjusts left motor pwm to drive at desired linear & angular velocity
void updateBumperStatus(void); //updates array with current bumper status 
void updateMotorStatus(void); //updates array with current motor status [directionL, pwmL, velocityL, directionR, pwmR, velocityR]

// ***************************************
// * Variable Decleration/Initialization *
// ***************************************

/* Used to determine elapsed time for velocity calculations */
unsigned long timePastVelocity=micros(),timePastUpdate=micros();

/* Incoming velocity commands from ROS*/
float speed_ang=0.0, speed_lin=0.0;

/* ROS odometry */
char odom[] = "/odom"; //stationary base world reference
char chassis[] = "/chassis"; //Bot location calculated using incoming velocity messages
char chassisEncoder[] = "/chassisEncoder"; //Bot location calculated using encoders

ros::NodeHandle  nh; //declare ROS nodeHandle

tf::TransformBroadcaster broadcaster; //broadcasts/publishes tf (where bot is)
geometry_msgs::TransformStamped u; //real location via Encoders
geometry_msgs::TransformStamped t; //Simulated location via /cmd_vel input

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb ); //cmd vel subscriber

std_msgs::Float32MultiArray motorStatus;  //motor status publisher
ros::Publisher motorStatus_pub("motorStatus", &motorStatus);

std_msgs::Int8MultiArray bumpers; //bumper status publisher
ros::Publisher bumpers_pub("bumpers", &bumpers);

void setup() {
  // put your setup code here, to run once:
  setupRSLK();
  nh.advertise(motorStatus_pub); //publish motor status
  nh.advertise(bumpers_pub); //publish bumper status
  nh.subscribe(sub); //subscribe to cmd vel
  nh.initNode();
  broadcaster.init(nh);
  
  delay(5000); //allow time to connect

  enableMotor(BOTH_MOTORS);
  timeInit(); //updates timer variables
  
}

void loop() {
  // put your main code here, to run repeatedly:
  
  /* How far each wheel has traveled since last loop */
  float rightDistanceTraveled = getDistanceTraveled(RIGHT_MOTOR); 
  float leftDistanceTraveled  = getDistanceTraveled(LEFT_MOTOR);

  /* Used if not sending cmd vel messages via cmd_vel*/
  speed_lin=-0.10;
  speed_ang=PI/4;

  /* Adjust motor PWM value to satsify desired linear/angular velocity */
  rightMotorControl(speed_lin,speed_ang);
  leftMotorControl(speed_lin,speed_ang);
  
  updateVelocity(rightDistanceTraveled, leftDistanceTraveled);

  updateLocationSim();
  updateLocationEncoder(leftDistanceTraveled, rightDistanceTraveled);

  updateBumperStatus();
  updateMotorStatus();
  
  nh.spinOnce();
  delay(125);
}

//**********************************************************************************
//Function Name: updateMotorStatus(void)
//Purpose: Publishes the motorStatus topic  
//Method: Stores key motor information in an array which is then published
//        *Note: LM = left motor, RM = right motor
//          motorStatusArray[LM Direction, LM PWM value, LM Wheel velocity...
//                             ...RM Direction, RM PWM value, RM Wheel Velocity]
//**********************************************************************************
void updateMotorStatus(void){
  float motorStatusArray[6]={0.0,0.0,0.0,0.0,0.0,0.0};

  /* Filling array with data*/
  motorStatusArray[0] = float(leftMotor.dir);  //left motor direction (-1=backward, 1=forward)
  motorStatusArray[1] = float(leftMotor.PWM);  //left motor PWM value [0,255]
  motorStatusArray[2] = leftMotor.currentVelocity;  //left motor linear velocity (m/s)
  motorStatusArray[3] = float(rightMotor.dir); //right motor direction (-1=backward, 1=forward)
  motorStatusArray[4] = float(rightMotor.PWM); //right motor PWM value [0,255]
  motorStatusArray[5] = rightMotor.currentVelocity; //right motor linear velocity (m/s)

  /* Setting data and publishing topic */
  motorStatus.data_length = 6;
  motorStatus.data = motorStatusArray;
  motorStatus_pub.publish(&motorStatus);
}

//**********************************************************************************
//Function Name: updateBumperStatus(void)
//Purpose: Updates and publishes bumpers topic
//Method: *The array bumper_reading[] stores the current state of each bumper switch
//        *Bumper pins (PIN_0 - PIN_5) are oriented on the rslk with PIN_0 on the right
//        *The array bumper_reading[] stores bumper states from the left most 
//          switch(PIN_5)to the right most switch(PIN_0)
//        *Bumper pins use INPUT_PULLUP so normally high, array value is set to 
//          !digitalRead(x)in order to follow true/false notation for is bumper pressed
//        *Now will display left to right 0-5 with 0 as not pressed and 1 as pressed 
//**********************************************************************************
void updateBumperStatus(void){

  int8_t bumper_reading[6] = {0,0,0,0,0,0};

  /* Filling array with data*/
  bumper_reading[0] = !digitalRead(BP_SW_PIN_5);
  bumper_reading[1] = !digitalRead(BP_SW_PIN_4);
  bumper_reading[2] = !digitalRead(BP_SW_PIN_3);
  bumper_reading[3] = !digitalRead(BP_SW_PIN_2);
  bumper_reading[4] = !digitalRead(BP_SW_PIN_1);
  bumper_reading[5] = !digitalRead(BP_SW_PIN_0);

  /* Setting data and publishing topic */
  bumpers.data_length = 6;
  bumpers.data = bumper_reading;
  bumpers_pub.publish(&bumpers);
}

//**********************************************************************************
//Function Name: getDistanceTraveled(int side)
//Purpose: Return linear distance traveled in meters for specified wheel since 
//         the function was last called
//Method: Read encoder counts for specified wheel and plug that count into a formula
//        that will calculae the linear distance traveled. The formula is based on 
//        physical characteristics of the vehicle. The encoder count is then reset. 
//        The encoder count will result in a negative value if the motor is moving
//        backwards
//**********************************************************************************
float getDistanceTraveled(int side){
  
  float distanceTraveled = 0.0;
  int encoderCount = 0;
  
  switch (side){
    case 0: //left motor
      encoderCount = getEncoderLeftCnt();
      resetLeftEncoderCnt();
      encoderCount = encoderCount * leftMotor.dir; //Negative if dir = backward
      break;
      
    case 1: //right motor
      encoderCount = getEncoderRightCnt();
      resetRightEncoderCnt();
      encoderCount = encoderCount * rightMotor.dir; //Negative if dir = backward
      break;
      
    default:
      encoderCount = 0;
      break;
  }

  /* Formula for encoder count to distance */
  distanceTraveled = (encoderCount * WHEEL_RAD * 2.0 * PI ) / CNT_PER_REVOLUTION;
  
  return distanceTraveled;
}

//**********************************************************************************
//Function Name: getTimeSinceLastUpdate(void)
//Purpose: Return time in seconds since function was last called
//Method: Global variable timePastUpdate maintains the time that the function was 
//        last called in micro seconds, when the function is called the timePastUpdate
//        is subracted from the current time resulting in the elapsed time since the
//        function was last called in micro seconds. The result is then multiplied by 
//        0.000001 to convert from micro seconds to seconds.
//        The current time is called via the built in Arduino function "micros()"
//**********************************************************************************
float getTimeSinceLastUpdate(void){
  
  unsigned long timeElapsedMicroSec = micros() - timePastUpdate;
  float timeElapsedSec = timeElapsedMicroSec * MULT_INV_ONE_MILL; //convert millisec to sec
  timePastUpdate = micros();
  return timeElapsedSec;
}

//**********************************************************************************
//Function Name: updateVelocity(float rightDistance, float leftDistance)
//Purpose: Updates global right and left wheel velocity variables
//Method: Takes in the distance traveled in meters for the right and left wheel
//        and divides the distance traveled by the time since this function was last
//        called in seconds, and updates the global variables rightVelocity, and 
//        leftVelocity with their respective velocities
//**********************************************************************************
void updateVelocity(float rightDistanceMeters, float leftDistanceMeters){
  
  unsigned long timeElapsedMicroSec = micros() - timePastVelocity;
  float timeElapsedSec = timeElapsedMicroSec * MULT_INV_ONE_MILL; //convert millisec to sec

  /* Update global velocity values */
  rightMotor.currentVelocity = rightDistanceMeters / timeElapsedSec;
  leftMotor.currentVelocity  = leftDistanceMeters  / timeElapsedSec;
  
  timePastVelocity=micros(); //update time 
}

//**********************************************************************************
//Function Name: updateLocationSim(void)
//Purpose: Maintain odometry information for the RSLK as if it drove/reacted exactly
//         as command velocity inputs instructed
//Method: Odometry info is updated via desired velocities (calculated in right/left 
//        motor control functions). Note that in order to find distance traveled 
//        since the function was last called the velocities must be multiplied by dt
//        which is the elapsed time since the function was last called in seconds.
//**********************************************************************************
void updateLocationSim(void){
  
  float dt = getTimeSinceLastUpdate();
  float baseVelocity = (rightMotor.desiredVelocity + leftMotor.desiredVelocity) * 0.5 * dt; //velocity * time = dist traveled

  /* Update x,y,theta: value= value + change since last update */
  simulated.theta +=  (((rightMotor.desiredVelocity - leftMotor.desiredVelocity) * dt) * (1.0 / (WHEEL_SEPERATION * 2.0))); 
  simulated.x     +=  (baseVelocity*cos(simulated.theta)) ;
  simulated.y     +=  (baseVelocity*sin(simulated.theta)) ;

  /* Update transform informaiton and then broadcast */
  t.header.frame_id = odom; //Stationary origin point/reference, this is the "parent"
  t.child_frame_id = chassis; //"Child" frame moving relative to "parent"

  /* Updating transform with new coordinate values */
  t.transform.translation.x = simulated.x;
  t.transform.translation.y = simulated.y;
  t.transform.rotation = tf::createQuaternionFromYaw(simulated.theta);
  t.header.stamp = nh.now();

  /* broadcaster publishes tf msgs */
  broadcaster.sendTransform(t);
}

//**********************************************************************************
//Function Name: updateLocationEncoder(float leftDist, float rightDist)
//Purpose: Maintain odometry information for the chassis based on the distance 
//         traveled by each wheel
//Method: Odometry info is updated via the distance traveled(function inputs) since 
//        the function was last called. 
//**********************************************************************************
void updateLocationEncoder(float leftDist, float rightDist){
  
  float distanceTraveled = (leftDist + rightDist) * 0.5;
  
  /* Update x,y,theta: value= value + change since last update */
  calculated.theta += (rightDist - leftDist) / (WHEEL_SEPERATION * 2);
  calculated.x     += (distanceTraveled*cos(calculated.theta)) ;
  calculated.y     += (distanceTraveled*sin(calculated.theta)) ;

  /* Update transform informaiton and then broadcast */
  u.header.frame_id = odom; //Stationary origin point/reference, this is the "parent"
  u.child_frame_id = chassisEncoder; //"Child" frame moving relative to "parent"

  /* Updating transform with new coordinate values */
  u.transform.translation.x = calculated.x;
  u.transform.translation.y = calculated.y;
  u.transform.rotation = tf::createQuaternionFromYaw(calculated.theta);
  u.header.stamp = nh.now();

  /* broadcaster publishes tf msgs */
  broadcaster.sendTransform(u);
}

//**********************************************************************************
//Function Name: timeInit(void)
//Purpose: Run direcly before main loop() to update all time variables to prevent
//         startup odom and velocity measurement errors
//Method: Runs all functions that are dependent on the elapsed time since last called
//**********************************************************************************
void timeInit(void){
  
  updateVelocity(0, 0); 
  getTimeSinceLastUpdate();
}

//**********************************************************************************
//Function Name: messageCb( const geometry_msgs::Twist& msg)
//Purpose: Recieve input commands from user
//Method: Updates desired linear and angular velocity messages via Twist message 
//        inputs from user. 
//        msg.linear.x  is in units meters per second
//        msg.angular.z is in units radians per second
//**********************************************************************************
void messageCb( const geometry_msgs::Twist& msg){
  
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
}

//**********************************************************************************
//Function Name: rightMotorControl(float linear, float angular)
//Purpose: To power the right wheel so that it drives at the correct speed in order 
//         to satisfy the incoming linear and angular velocity commands
//Method: Takes in desired linear and angular velocity as arguments and sets motor 
//        PWM/speed as follows:
//          1. Determins the linear velocity that the left wheel must be in order 
//             to satisfy overall velocity requirements. 
//          2. Calculate error (how far off the current velocity is vs desired)
//          3. The motor direction is determined and set based on the required velocity.
//          4. If direction is reverse then error is multiplied by -1 via motor.dir
//          5. Runs through control loop to calculate new motor speed value [0,255]
//                *Motor speed change is proportional to error value
//          6. Sets motor speed to new value based on control loop results
//**********************************************************************************
void rightMotorControl(float linear, float angular){
  
  /* Formula to desire velocity in m/s in relation to twist linear&angular values */ 
  rightMotor.desiredVelocity = linear + (angular * WHEEL_SEPERATION) ;
  
  /* Determine motor direction */ 
  if (rightMotor.desiredVelocity >= 0){ //if desired velocity positive
    rightMotor.dir = FORWARD; //set struct motor direction forward (1)
    setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_FORWARD); //set motor to forward (0)
    }
  else {  //desired velocity is negative
    rightMotor.dir = BACKWARD; //set struct motor direction backward (-1)
    setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_BACKWARD);//set motor direction backward (1)
    }

  /* Error is the difference between desired and current velocity */ 
  float error = (rightMotor.desiredVelocity - rightMotor.currentVelocity) * rightMotor.dir;

  /*Adjust motor PWM in proportion to error value*/ 
  rightMotor.PWM += int8_t(error * PROPORTIONAL_CORRECTION);

  /*Check to ensure PWM is within range [0,255] */ 
  if (rightMotor.PWM > MAX_MOTOR_PWM){rightMotor.PWM = MAX_MOTOR_PWM;}
  
  /*Setting motor speed*/
  setRawMotorSpeed(RIGHT_MOTOR , rightMotor.PWM);
}

//**********************************************************************************
//Function Name: leftMotorControl(float linear, float angular)
//Purpose: To power the left wheel so that it drives at the correct speed in order 
//         to satisfy the incoming linear and angular velocity commands
//Method: Takes in desired linear and angular velocity as arguments and sets motor 
//        PWM/speed as follows:
//          1. Determins the linear velocity that the left wheel must be in order 
//             to satisfy overall velocity requirements. 
//          2. Calculate error (how far off the current velocity is vs desired)
//          3. The motor direction is determined and set based on the required velocity.
//          4. If direction is reverse then error is multiplied by -1 via motor.dir
//          5. Runs through control loop to calculate new motor speed value [0,255]
//                *Motor speed change is proportional to error value
//          6. Sets motor speed to new value based on control loop results
//**********************************************************************************
void leftMotorControl(float linear, float angular){
 
  /* Formula to desire velocity in m/s in relation to twist linear&angular values */ 
  leftMotor.desiredVelocity = linear - (angular * WHEEL_SEPERATION) ;
  
  /* Determine motor direction */ 
  if (leftMotor.desiredVelocity >= 0){ //if desired velocity positive
    leftMotor.dir = FORWARD; //set struct motor direction forward (1)
    setMotorDirection(LEFT_MOTOR,MOTOR_DIR_FORWARD); //set motor to forward (0)
    }
  else {  //desired velocity is negative
    leftMotor.dir = BACKWARD; //set struct motor direction backward (-1)
    setMotorDirection(LEFT_MOTOR,MOTOR_DIR_BACKWARD); //set motor direction backward (1)
    }
  
  /* Error is the difference between desired and current velocity */ 
  float error = (leftMotor.desiredVelocity - leftMotor.currentVelocity) * leftMotor.dir;

  /*Adjust motor PWM in proportion to error value*/ 
  leftMotor.PWM += int8_t(error * PROPORTIONAL_CORRECTION);

  /*Check to ensure PWM is within range [0,255] */ 
  if (leftMotor.PWM > MAX_MOTOR_PWM){leftMotor.PWM = MAX_MOTOR_PWM;}

  /*Setting motor speed*/
  setRawMotorSpeed(LEFT_MOTOR , leftMotor.PWM); 
}
