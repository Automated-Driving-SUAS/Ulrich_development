#include "imxrt.h"
#include "robot.h"

//The serial data address for roboclaw motor driver
#define address 0x80

//The robot class object constructor 
Robot::Robot(double wheelRadius, double trackWidth, double maxRPM){
    this->wheelRadius = wheelRadius;
    this->trackWidth = trackWidth;
    this->maxRPM = maxRPM;
    maxVel = (maxRPM*2*PI*wheelRadius)/60;
}

//Encoder intializatin fucntion
void Robot::EncodersInit(QuadEncoder *leftEncoder, QuadEncoder *rightEncoder , double encoderRes){
    this->encoderRes = encoderRes;
    this->leftEncoder = leftEncoder;
    this->rightEncoder = rightEncoder;
    leftEncoder->setInitConfig();
    leftEncoder->init();
    rightEncoder->setInitConfig();
    rightEncoder->init();  
}

//Odometry msg intialization
void Robot::OdometryInit(nav_msgs__msg__Odometry *odom_msg){
    this->odom_msg = odom_msg;
    odom_msg->header.frame_id = micro_ros_string_utilities_set(odom_msg->header.frame_id, "odom");
    odom_msg->child_frame_id = micro_ros_string_utilities_set(odom_msg->child_frame_id, "base_link");
}


//Roboclaw motor driver intialization
void Robot::motorsInit(RoboClaw *roboclaw, long baudrate)
{
    this->roboclaw = roboclaw;
    this->baudrate = baudrate;
    this->roboclaw->begin(baudrate);
}

//Updating actuall velocity from the encoder counts 
void Robot::getRobotVelocity(){

  unsigned long current_time = micros();
	unsigned long dt = current_time - prev_update_time;

  this->updateTime = (double)dt/1000000;

  if(abs(leftEncoder->read())< 50){
    velActual.left = 0;
  }
  else {
    velActual.left = (((double)leftEncoder->read()/encoderRes)*wheelRadius*PI*2)/updateTime;
  }  
  if(abs(rightEncoder->read())< 50){
    velActual.right = 0;
  }
  else {
    velActual.right = (((double)rightEncoder->read()/encoderRes)*wheelRadius*PI*2)/updateTime;
  }
  leftEncoder->write(0);
  rightEncoder->write(0);

  prev_update_time = current_time;
}

//Updating odometry data from the actual velocity
void Robot::updateOdometryData(){
      getRobotVelocity();

      //calculating the change in linear velocity and angular velocity
      float dv = ((velActual.right+velActual.left)*(updateTime))/2;
      float dTheta = ((velActual.right-velActual.left)*(updateTime))/trackWidth;

      //Calculating the change in postion in x and y coordinates  
      float dx= cos(dTheta)*dv;
      float dy= sin(dTheta)*dv;

      //Calculating the cummulative position change
      xPos += (cos(theta)*dx - sin(theta)*dy);
      yPos += (cos(theta)*dx + sin(theta)*dy);
      theta += dTheta;

      //Reseting the angle after a complete rotation  
      if(theta >= TWO_PI) theta -= TWO_PI;
      if(theta <= -TWO_PI) theta += TWO_PI;

      //changing the euler data to quaternion
      double q[4];
      euler_to_quat(0,0,theta,q);

      //robot position in x, y, z;
      odom_msg->pose.pose.position.x = xPos;
      odom_msg->pose.pose.position.y = yPos;
      odom_msg->pose.pose.position.z = 0.0;

      //robot's heading in quaternion
      odom_msg->pose.pose.orientation.x = q[1];
      odom_msg->pose.pose.orientation.y = q[2];
      odom_msg->pose.pose.orientation.z = q[3];
      odom_msg->pose.pose.orientation.w = q[0];

      //linear speed from encoders
      odom_msg->twist.twist.linear.x = (velActual.right+velActual.left)/2;

      //angular speed from encoders
      odom_msg->twist.twist.angular.z = (velActual.right-velActual.left)/trackWidth;
}


//Move the robot according the cmd_velocity
void Robot::moveRobot(geometry_msgs__msg__Twist *cmdvel_msg, unsigned long prev_cmd_time){
    // braking if there's no command received after 200ms
    if((millis()-prev_cmd_time)>=300){
      velReq.left = 0.0;
      velReq.right = 0.0;
    }
    else{
      velReq.left = (cmdvel_msg->linear.x + (cmdvel_msg->angular.z*(trackWidth/2)));
      velReq.right = (cmdvel_msg->linear.x - (cmdvel_msg->angular.z*(trackWidth/2)));
    }
    Vel value;
    value.left = map(constrain(velReq.left, -maxVel , maxVel), -maxVel , maxVel , -127, 127);
    value.right = map(constrain(velReq.right, -maxVel , maxVel), -maxVel , maxVel , -127, 127);
    if(value.left >= 0)
        roboclaw->ForwardM1(address, value.left);
    else
        roboclaw->BackwardM1(address, -value.left);
    
    if(value.right >= 0)
        roboclaw->ForwardM2(address, value.right);
    else
        roboclaw->BackwardM2(address, -value.right);
}

//IMU intialize function
void Robot::imuBNO055_init(Adafruit_BNO055* bno){
    bno->begin();

    //Calibration data update from EEPROM
    int eeAddress = 0;
    long bnoID;
    EEPROM.get(eeAddress, bnoID);
    adafruit_bno055_offsets_t calibrationData;
    sensor_t bnoSensor;
    bno->getSensor(&bnoSensor);
    if(bnoID == (bnoSensor.sensor_id)){
        eeAddress+= sizeof(long);
        EEPROM.get(eeAddress, calibrationData);
        bno->setSensorOffsets(calibrationData);
    }
}


//Updating the IMU data
void Robot::updateImuBNO055Data(Adafruit_BNO055* bno, sensor_msgs__msg__Imu* imuMsg){

    sensors_event_t angVelocityData, linearAccelData;
    bno->getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno->getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Quaternion quat = bno->getQuat();
  
    imuMsg->header.frame_id = micro_ros_string_utilities_set(imuMsg->header.frame_id, "imu_link");

    imuMsg->linear_acceleration.x = linearAccelData.acceleration.x;
    imuMsg->linear_acceleration.y = linearAccelData.acceleration.y;
    imuMsg->linear_acceleration.z = linearAccelData.acceleration.z;

    imuMsg->angular_velocity.x = angVelocityData.gyro.x;
    imuMsg->angular_velocity.y = angVelocityData.gyro.y;
    imuMsg->angular_velocity.z = angVelocityData.gyro.z;

    imuMsg->orientation.w =  quat.w();
    imuMsg->orientation.x =  quat.x();
    imuMsg->orientation.y =  quat.y();
    imuMsg->orientation.z =  quat.z();

    imuMsg->orientation_covariance[0] = -1;
    imuMsg->linear_acceleration_covariance[0] = -1;
    imuMsg->angular_velocity_covariance[0] = -1;

}

//roll, yaw and pitch are in rad/sec

const void Robot::euler_to_quat(float roll, float pitch, float yaw, double *q) 
{
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    q[0] = cy * cp * cr + sy * sp * sr;
    q[1] = cy * cp * sr - sy * sp * cr;
    q[2] = sy * cp * sr + cy * sp * cr;
    q[3] = sy * cp * cr - cy * sp * sr;
}