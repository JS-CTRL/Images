#include <ros.h> //required for ROS
#include <ros/time.h> //required for ROS odom
#include <tf/tf.h> //required for ROS odom
#include <tf/transform_broadcaster.h> //required for ROS odom
#include "SimpleRSLK.h" //contains RSLK functions
#include <std_msgs/Float32.h> //Floats used for ROS messages
#include <geometry_msgs/Twist.h>//Twist used for ROS incoming velocity messages


#define WHEEL_SEPERATION 0.07 /* Distance between wheels in meters */
#define WHEEL_RAD 0.035 //more accurate 0.03475 /* Radius of RSLK wheels in meters */
#define CNT_PER_REVOLUTION 360 /* Number of encoder (rising) pulses per wheel revolution */
#define MAIN_DELAY 75 //delay in main loop-used as variable in testing *Can be deleted
#define VARIABLE 1 //changes theta equation depending on source *Can be deleted


/* Function Prototypes*/
double getDistanceTraveled(int side);
double getTimeSinceLastUpdate(void);
double getTimeSinceLastUpdate2(void);
void updateVelocity(double rightDistance, double leftDistance);
void updateLocationSim(void);
void updateLocationVelocity(void);
void updateLocationEncoder(double leftDist, double rightDist);
void timeInit(void);
void messageCb( const geometry_msgs::Twist& msg);
void rightMotorControl(double linear, double angular);
void leftMotorControl(double linear, double angular);


// ***************************************
// * Variable Decleration/Initialization *
// ***************************************

/* Direction of each motor */
int rightMotorDir=MOTOR_DIR_FORWARD;
int leftMotorDir=MOTOR_DIR_FORWARD;

/* Used to determine elapsed time for velocity calculations */
unsigned long timePastVelocity=micros(),timePastUpdate=micros(),timePastUpdate2=micros();

/* Current linear velocity of each wheel */
double rightVelocity=0.0, leftVelocity = 0.0;

/* Current odometry readings */
float x=0,  y=0, theta=0; //Simulated "perfect"
float x2=0,  y2=0, theta2=0; //Using Encoders measurements 
float x3=0,  y3=0, theta3=0; //Using Velocity measurements

/* Incoming velocity commands from ROS*/
float speed_ang=0, speed_lin=0;

/* Calculated motor velocity to satisfy Incoming velocity commands from ROS */
float desiredVelocityLeft = 0.0   , desiredVelocityRight = 0.0;

/* Current motor value (0-255) */
int motorSpeedValueRight=0, motorSpeedValueLeft=0;

/* ROS odometry */
char odom[] = "/odom"; //base link 
char chassis[] = "/chassis"; //Simulated Bot
char chassisReal[] = "/chassisReal"; //Bot using encoders
char chassisVelocity[] = "/chassisVelocity"; //bot using velocity

//Starts ROS
ros::NodeHandle  nh;


tf::TransformBroadcaster broadcaster;
geometry_msgs::TransformStamped u; //real location via Encoders
geometry_msgs::TransformStamped v; //real location via Velocity
geometry_msgs::TransformStamped t; //Simulated location via /cmd_vel input

std_msgs::Float32 right_wheel_velocity;
ros::Publisher rightWheelPub("right_wheel_velocity", &right_wheel_velocity);

std_msgs::Float32 left_wheel_velocity;
ros::Publisher leftWheelPub("left_wheel_velocity", &left_wheel_velocity);

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  setupRSLK();

  nh.advertise(leftWheelPub);
  nh.advertise(rightWheelPub); //
  nh.subscribe(sub); //command velocity
  nh.initNode();
  broadcaster.init(nh);
  delay(5000);
  
  enableMotor(BOTH_MOTORS);
  timeInit();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  double rightDistanceTraveled = getDistanceTraveled(RIGHT_MOTOR);
  double leftDistanceTraveled  = getDistanceTraveled(LEFT_MOTOR);
  
  speed_lin=0.1;
  speed_ang=0;

  rightMotorControl(speed_lin,speed_ang);
  leftMotorControl(speed_lin,speed_ang);


 
  
  updateVelocity(rightDistanceTraveled, leftDistanceTraveled);

  updateLocationSim();
  //updateLocationEncoder(leftDistanceTraveled, rightDistanceTraveled);
  updateLocationVelocity();


  nh.spinOnce();
  delay(MAIN_DELAY);
  


/*
  Serial.print("Goal:"); Serial.print(speed_lin*100);     Serial.print(",");
  Serial.print("LV:"); Serial.print(leftVelocity*100);     Serial.print(",");
  Serial.print("LS:"); Serial.print(motorSpeedValueLeft);     Serial.print(",");
  Serial.print("RV:"); Serial.print(rightVelocity*100);     Serial.print(",");
  Serial.print("RS:"); Serial.print(motorSpeedValueRight);     Serial.print(",");
  Serial.println();
*/



/*
  Serial.print("x:"); Serial.print(x);     Serial.print(",");
  Serial.print("y:"); Serial.print(y);     Serial.print(",");
  Serial.print("@:"); Serial.print(theta); Serial.print(",");
  Serial.println();
*/
/*
  Serial.print("x:");    Serial.print(x);
  Serial.print("  y:");  Serial.print(y);
  Serial.print("  @:");  Serial.print(theta);

  Serial.print("  LV:"); Serial.print(leftVelocity);
//  Serial.print("  LS:"); Serial.print(motorSpeedValueLeft);
  Serial.print("  RV:"); Serial.print(rightVelocity);
//  Serial.print("  RS:"); Serial.println(motorSpeedValueRight);
  
  Serial.println();
  */
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
double getDistanceTraveled(int side){
  double distanceTraveled=0.0;
  int encoderCount=0;
  
  switch (side){
    case 0: //left 
      encoderCount=getEncoderLeftCnt();
      resetLeftEncoderCnt();
      if (leftMotorDir==1){//if dir is reverse then encoder negative
        encoderCount = encoderCount * -1;
      }
      break;
      
    case 1: //right
      encoderCount=getEncoderRightCnt();
      resetRightEncoderCnt();
      if (rightMotorDir==1){//if dir is reverse then encoder negative
        encoderCount = encoderCount * -1;
      }
      break;
    default:
      encoderCount=0;
      break;
  }
  
  distanceTraveled=(encoderCount * WHEEL_RAD * 2.0 * PI ) / CNT_PER_REVOLUTION;
  
  return distanceTraveled;
}// end getDistanceTraveled(int side)


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
double getTimeSinceLastUpdate(void){
  unsigned long timeElapsedMicroSec=micros()-timePastUpdate;
  double timeElapsedSec=timeElapsedMicroSec*0.000001; //convert millisec to sec
  timePastUpdate=micros();
  return timeElapsedSec;
}


//**********************************************************************************
//Function Name: getTimeSinceLastUpdate2(void)
//Purpose: Return time in seconds since function was last called
//Method: Global variable timePastUpdate maintains the time that the function was 
//        last called in micro seconds, when the function is called the timePastUpdate2
//        is subracted from the current time resulting in the elapsed time since the
//        function was last called in micro seconds. The result is then multiplied by 
//        0.000001 to convert from micro seconds to seconds.
//        The current time is called via the built in Arduino function "micros()"
//**********************************************************************************
double getTimeSinceLastUpdate2(void){
  unsigned long timeElapsedMicroSec=micros()-timePastUpdate2;
  double timeElapsedSec=timeElapsedMicroSec*0.000001; //convert millisec to sec
  timePastUpdate2=micros();
  return timeElapsedSec;
}

//**********************************************************************************
//Function Name: updateVelocity(double rightDistance, double leftDistance)
//Purpose: Updates global right and left wheel velocity variables
//Method: Takes in the distance traveled in meters for the right and left wheel
//        and divides the distance traveled by the time since this function was last
//        called in seconds, and updates the global variables rightVelocity, and 
//        leftVelocity with their respective velocities
//**********************************************************************************
void updateVelocity(double rightDistance, double leftDistance){
  double leftDistanceMeters = leftDistance;
  double rightDistanceMeters = rightDistance;
  unsigned long timeElapsedMicroSec=micros()-timePastVelocity;
  double timeElapsedSec=timeElapsedMicroSec*0.000001; //convert millisec to sec

  rightVelocity=rightDistanceMeters/timeElapsedSec; //distance in M divided by time in S
  leftVelocity=leftDistanceMeters/timeElapsedSec;

  left_wheel_velocity.data=leftVelocity;
  leftWheelPub.publish(&left_wheel_velocity);

  right_wheel_velocity.data=rightVelocity;
  rightWheelPub.publish(&right_wheel_velocity);

  timePastVelocity=micros();//update time 

/*
  Serial.print("Left_Velocity:");
  Serial.print(leftVelocity);
  Serial.print("Right_Velocity:");
  Serial.print(rightVelocity);
*/

/*
  Serial.print("\nLeft Velocity:");
  Serial.print(leftVelocity);
  Serial.print("  Left Drive Value:");
  Serial.print(motorSpeedValueLeft);
  Serial.print("  Right Velocity:");
  Serial.print(rightVelocity);
  Serial.print("  Right Drive Value:");
  Serial.println(motorSpeedValueRight);
 */ 
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
  double dt=getTimeSinceLastUpdate();
  double baseVelocity=(desiredVelocityRight+desiredVelocityLeft)*0.5*dt;

  theta +=  ( ( (desiredVelocityRight-desiredVelocityLeft) *dt ) * (1.0/(WHEEL_SEPERATION*VARIABLE)) )  ; 

  x     +=  (baseVelocity*cos(theta)) ;
  y     +=  (baseVelocity*sin(theta)) ;

  t.header.frame_id = odom;
  t.child_frame_id = chassis;

  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.rotation = tf::createQuaternionFromYaw(theta);
  t.header.stamp = nh.now();
  
  broadcaster.sendTransform(t);
}

//**********************************************************************************
//Function Name: updateLocationVelocity(void)
//Purpose: Maintain odometry information based on measured velocity
//Method: Odometry info is updated via measured velocity which globally updated via
//        the updateVelocity() function. Note that in order to find distance traveled 
//        since the function was last called the velocities must be multiplied by dt
//        which is the elapsed time since the function was last called in seconds.
//**********************************************************************************
void updateLocationVelocity(void){
  double dt=getTimeSinceLastUpdate2();
  double baseVelocity=(rightVelocity+leftVelocity)*0.5*dt;
  theta3 +=  ( ( (rightVelocity-leftVelocity) * dt) * (1.0/(VARIABLE*WHEEL_SEPERATION) )) ;
  x3     +=  (baseVelocity*cos(theta3)) ;
  y3     +=  (baseVelocity*sin(theta3)) ;

  v.header.frame_id = odom;
  v.child_frame_id = chassisVelocity;

  v.transform.translation.x = x3;
  v.transform.translation.y = y3;
  v.transform.rotation = tf::createQuaternionFromYaw(theta3);
  v.header.stamp = nh.now();
  
  broadcaster.sendTransform(v);
}
//**********************************************************************************
//Function Name: updateLocationEncoder(double leftDist, double rightDist)
//Purpose: Maintain odometry information for the chassis based on the distance 
//         traveled by each wheel
//Method: Odometry info is updated via the distance traveled(function inputs) since 
//        the function was last called. 
//**********************************************************************************
void updateLocationEncoder(double leftDist, double rightDist){


  
  double distanceTraveled= (leftDist + rightDist ) * 0.5;
  
  double dTheta = (rightDist - leftDist) / (  WHEEL_SEPERATION*VARIABLE);
  theta2 += dTheta;
  
  double dx = (distanceTraveled*cos(theta2)) ;
  x2 += dx;
  
  double dy =  (distanceTraveled*sin(theta2)) ;
  y2 += dy;
  
  u.header.frame_id = odom;
  u.child_frame_id = chassisEncoder;
  
  u.transform.translation.x = x2;
  u.transform.translation.y = y2;
  u.transform.rotation = tf::createQuaternionFromYaw(theta2);
  u.header.stamp = nh.now();
  
  broadcaster.sendTransform(u);

  
  /*
  Serial.print(leftDistanceTraveled);
  Serial.print("  ");
  Serial.println(rightDistanceTraveled);
  */
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
  getTimeSinceLastUpdate2();
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
//Function Name: rightMotorControl(double linear, double angular)
//Purpose: To power the right wheel so that it drives at the correct speed in order 
//         to satisfy the incoming linear and angular velocity commands
//Method: Takes in desired linear and angular velocity as arguments and sets motor 
//        speed as follows:
//          1. Determins the linear velocity that the right wheel must be in order 
//             to satisfy overall velocity requirements. 
//          2. The motor direction is determined and set based on the required velocity. 
//          3. Calculate error (how far off the current velocity is vs desired)
//          4. Runs through control loop to calculate new motor speed value (0-255)
//                *Motor speed change is proportional to error value
//          5. Sets motor speed to new value based on control loop results
//**********************************************************************************
void rightMotorControl(double linear, double angular){
  float error=0.0;
  float kp=0.6;
  
  //Formula to desire velocity in m/s in relation to twist linear&angular values
  desiredVelocityRight=  ( (2*linear) + (angular * WHEEL_SEPERATION) ) / 2;
  
  //Determining Motor Direction 
  if (desiredVelocityRight >= 0){rightMotorDir=MOTOR_DIR_FORWARD;}
  else rightMotorDir=MOTOR_DIR_BACKWARD;
  

  setMotorDirection(RIGHT_MOTOR,rightMotorDir);

  
  error=(desiredVelocityRight-rightVelocity)*100.0;
   
  if ((abs(motorSpeedValueRight)+abs(int(error*kp))) < 200){
    if (rightMotorDir == 0){ //use 200 to cap motor top speed
      motorSpeedValueRight+=int(error*kp);
      }
    else if (rightMotorDir == 1){
      motorSpeedValueRight-= int(error*kp);
    }
  }
  
  setRawMotorSpeed(RIGHT_MOTOR,motorSpeedValueRight); //Setting motor speed (0-255)
/*
  Serial.print("desired:"); Serial.print(desiredVelocityRight);     Serial.print(",");
  Serial.print("actual:"); Serial.print(rightVelocity);     Serial.print(",");
  Serial.print("error:"); Serial.print(error); Serial.print(",");
  Serial.print("motorSpeedValue:"); Serial.print(motorSpeedValueRight); Serial.print(",");
  Serial.println();
 */
}
//**********************************************************************************
//Function Name: leftMotorControl(double linear, double angular)
//Purpose: To power the left wheel so that it drives at the correct speed in order 
//         to satisfy the incoming linear and angular velocity commands
//Method: Takes in desired linear and angular velocity as arguments and sets motor 
//        speed as follows:
//          1. Determins the linear velocity that the left wheel must be in order 
//             to satisfy overall velocity requirements. 
//          2. The motor direction is determined and set based on the required velocity. 
//          3. Calculate error (how far off the current velocity is vs desired)
//          4. Runs through control loop to calculate new motor speed value (0-255)
//                *Motor speed change is proportional to error value
//          5. Sets motor speed to new value based on control loop results
//**********************************************************************************
void leftMotorControl(double linear, double angular){
  float kp=0.6;
  kp=kp*100; //increases value for later so that it is large enough to influence speed value
  //Formula to desire velocity in m/s in relation to twist linear&angular values
  desiredVelocityLeft =  ( (2.0 * linear) - (angular * WHEEL_SEPERATION) ) / 2.0;
 
  //Determining Motor Direction
  if (desiredVelocityLeft >= 0){leftMotorDir=MOTOR_DIR_FORWARD;}
  else leftMotorDir=MOTOR_DIR_BACKWARD;
  
  setMotorDirection(LEFT_MOTOR,leftMotorDir);

  float error=(desiredVelocityLeft-leftVelocity);//too slow error is pow

  if ( (abs(motorSpeedValueLeft)+abs(int(error*kp))) < 200 ){
    if (leftMotorDir==0){ //use 200 to cap motor top speed
      motorSpeedValueLeft+=int(error*kp);
      }
    else if (leftMotorDir==1){
      motorSpeedValueLeft-=int(error*kp);
    }
  }
  
  setRawMotorSpeed(LEFT_MOTOR,motorSpeedValueLeft); //Setting motor speed (0-255)
  /*
  Serial.print("desired:"); Serial.print(desiredVelocityLeft);     Serial.print(",");
  Serial.print("actual:"); Serial.print(leftVelocity);     Serial.print(",");
  Serial.print("error:"); Serial.print(error); Serial.print(",");
  Serial.print("motorSpeedValueLeft:"); Serial.print(motorSpeedValueLeft); Serial.print(",");
  Serial.println();
  */
}
