/* Included for const PI */
#include <Arduino.h>


/* Distance between wheels in meters */
#define WHEEL_SEPERATION 0.07

/* Radius of RSLK wheels in meters */
#define WHEEL_RAD 0.035 //more accurate 0.03475

/* Number of encoder (rising) pulses per wheel revolution */
#define CNT_PER_REVOLUTION 360

/* Variable to use in equation: (CONVERSION_FACTOR * pulse count) = distance traveled in meters*/
#define CONVERSION_FACTOR ((2*WHEEL_RAD*3.14159265359)/CNT_PER_REVOLUTION)



#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>


#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "SimpleRSLK.h" //contains RSLK functions

#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
//Starts ROS
ros::NodeHandle  nh;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

double x = 0.0;
double y = 0.0;
double theta = 1.57;

char base_link[] = "/base_link";
char odom[] = "/odom";

int motorSpeedValueRight=0;
float desiredVelocityRight=0;

int motorSpeedValueLeft=0;
float desiredVelocityLeft=0;
float leftVelocity  = 0.0;
float rightVelocity = 0.0;


std_msgs::Float32 right_wheel_velocity;
ros::Publisher rightWheelPub("right_wheel_velocity", &right_wheel_velocity);

std_msgs::Float32 left_wheel_velocity;
ros::Publisher leftWheelPub("left_wheel_velocity", &left_wheel_velocity);


double speed_ang=0, speed_lin=0;//declare and initialize twist variables 

//float left__wheel_velocity = 0, right__wheel_velocity = 0; //to publish wheel velocity


void updateVelocity(void);

void rightMotorControl(double linear, double angular);
void leftMotorControl(double linear, double angular);
void messageCb( const geometry_msgs::Twist& msg);








ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );

void setup()
{ 
//Serial.begin(57600);
  /* Run setup code */
  setupRSLK();
  nh.advertise(leftWheelPub);
  nh.advertise(rightWheelPub);
  
  
  nh.subscribe(sub);
  
  nh.initNode();
  broadcaster.init(nh);
  
  delay(5000);
 // Serial1.begin(115200);
 // Serial1.println("Setup Complete");
}


float timeDiff=0;
float timeThen=millis();
float timeNow=0;
float sec=0.0;
float baseVelocity=0.0;

void loop()
{  

// For checking connectivity
  
  while (!nh.connected()){ //does not appear to stop motors
  
    setMotorSpeed(BOTH_MOTORS,0);
    disableMotor(BOTH_MOTORS);
    ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );
    setupRSLK();
  
    nh.subscribe(sub);
    nh.initNode();
  
    delay(5000);
    //Serial.println("Setup Complete");
    nh.spinOnce();
    delay(1);
  }


  /* "Turn on" the motor */
  enableMotor(BOTH_MOTORS);

 updateVelocity();


  rightMotorControl(speed_lin,speed_ang);
  leftMotorControl(speed_lin,speed_ang);



theta +=  ( ( rightVelocity-leftVelocity ) * (WHEEL_RAD/WHEEL_SEPERATION ) ) ;
  
 // if(theta > 3.14) theta=-3.14;
 if (theta > 3.14)  theta = -3.14;
 if (theta < -3.14) theta = 3.14;
  

baseVelocity=(rightVelocity+leftVelocity)*0.5*WHEEL_RAD;
x     +=  (baseVelocity*cos(theta)) ;
y     +=  (baseVelocity*sin(theta)) ;

// tf odom->base_link
  t.header.frame_id = odom;
  t.child_frame_id = base_link;


  
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  //t.transform.rotation.z = theta;
  t.transform.rotation = tf::createQuaternionFromYaw(theta);
  t.header.stamp = nh.now();
  
  broadcaster.sendTransform(t);

  nh.spinOnce();


  //delay(10);


}




/* Gets velocity messages from User*/
void messageCb( const geometry_msgs::Twist& msg){
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
}


void leftMotorControl(double linear, double angular){
  float error=0;
  float kp=0.75;
  //Formula to calculate desire velocity in m/s in relation to twist linear&angular values
  desiredVelocityLeft=  ( (2*linear) - (angular * WHEEL_SEPERATION) ) / 2;
  //Determining Motor Direction
  if (linear  < 0){setMotorDirection(LEFT_MOTOR,MOTOR_DIR_BACKWARD);}
  
  else if (linear > 0){setMotorDirection(LEFT_MOTOR,MOTOR_DIR_FORWARD);}

  desiredVelocityLeft=abs(desiredVelocityLeft);
  
  //Setting motor speed (0-255)
  setRawMotorSpeed(LEFT_MOTOR,motorSpeedValueLeft);

  //Speed Control loop
  error=(desiredVelocityLeft-leftVelocity)*100;//too slow error is pos

  motorSpeedValueLeft+=int(error*kp);
        
  if (abs(motorSpeedValueLeft)<200){
    motorSpeedValueLeft+=int(error*kp);
  }

    if (desiredVelocityLeft==0){
    motorSpeedValueLeft=0;
  }
}

void rightMotorControl(double linear, double angular){
  float error=0;
  float kp=0.75;
  //Formula to desire velocity in m/s in relation to twist linear&angular values
  desiredVelocityRight =  ( (2*linear) + (angular * WHEEL_SEPERATION) ) / 2;
  //Determining Motor Direction
  if (linear  < 0){setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_BACKWARD);}
  
  else if (linear > 0){setMotorDirection(RIGHT_MOTOR,MOTOR_DIR_FORWARD);}

  desiredVelocityRight=abs(desiredVelocityRight);
  
  //Setting motor speed (0-255)
  setRawMotorSpeed(RIGHT_MOTOR,motorSpeedValueRight);

  //Speed Control loop
  error=(desiredVelocityRight-rightVelocity)*100;//too slow error is pow

 motorSpeedValueRight+=int(error*kp);
        
  if (abs(motorSpeedValueRight)<200){
    motorSpeedValueRight+=int(error*kp);
  }

  if (desiredVelocityRight==0){
    motorSpeedValueRight=0;
  }

}





void updateVelocity(void){
  float calculatedSpeed=0;
  int measurementInterval=100000; //how much time to measure encoder count
  float previousTime=0.0;

  resetRightEncoderCnt();
  resetLeftEncoderCnt();
  
  previousTime= micros();

  /*
  while (elapsedTime < measurementInterval){
    elapsedTime=micros()-previousTime;
  }
  */
  while ( micros() < (previousTime+measurementInterval));

  leftVelocity=(CONVERSION_FACTOR*getEncoderLeftCnt())*(1000000.0/measurementInterval);
  rightVelocity=(CONVERSION_FACTOR*getEncoderRightCnt())*(1000000.0/measurementInterval);
  
  left_wheel_velocity.data=leftVelocity;
  leftWheelPub.publish(&left_wheel_velocity);

  right_wheel_velocity.data=rightVelocity;
  rightWheelPub.publish(&right_wheel_velocity);
  
}//end updateVelocity
