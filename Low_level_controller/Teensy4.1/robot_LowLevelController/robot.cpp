#include "imxrt.h"
#include "robot.h"

//The serial data address for roboclaw motor driver
#define address 0x80

//The robot class object constructor 
Robot::Robot(float wheelRadius = 0.0, float trackWidth = 0.0, float maxRPM = 0.0){
    this->wheelRadius = wheelRadius;
    this->trackWidth = trackWidth;
    this->maxRPM = maxRPM;
    maxVel = (maxRPM*2*PI*wheelRadius)/60;
}

//Encoder 

void Robot::EncodersInit(QuadEncoder *leftEncoder, QuadEncoder *rightEncoder ,unsigned int encoderRes){
    this->encoderRes = encoderRes;
    this->leftEncoder = leftEncoder;
    this->rightEncoder = rightEncoder;
    leftEncoder->setInitConfig();
    leftEncoder->init();
    rightEncoder->setInitConfig();
    rightEncoder->init();  
}

void Robot::getRobotVelocity(){
    velActual.left = (((float)leftEncoder->read()/encoderRes)*wheelRadius*PI*2)*(1000/updateTime);
    velActual.right = (((float)rightEncoder->read()/encoderRes)*wheelRadius*PI*2)*(1000/updateTime);
    leftEncoder->write(0);
    rightEncoder->write(0);
}

void Robot::OdometryInit(nav_msgs__msg__Odometry *odom_msg, unsigned int updateTime){
    this->odom_msg = odom_msg;
    this->updateTime = updateTime;
    odom_msg->header.frame_id = micro_ros_string_utilities_set(odom_msg->header.frame_id, "odom");
    odom_msg->child_frame_id = micro_ros_string_utilities_set(odom_msg->child_frame_id, "base_link");
}

void Robot::updateOdometry(){
        getRobotVelocity();
        float dv = ((velActual.right+velActual.left)*(1000/updateTime))/2;
        float dTheta = ((velActual.right-velActual.left)*(1000/updateTime))/trackWidth;

        float dx= cos(dTheta)*dv;
        float dy= sin(dTheta)*dv;

        xPos += (cos(theta)*dx - sin(theta)*dy);
        yPos += (cos(theta)*dx + sin(theta)*dy);
        theta += dTheta;

        if(theta >= TWO_PI) theta -= TWO_PI;
        if(theta <= -TWO_PI) theta += TWO_PI;
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

void Robot::motorsInit(RoboClaw *robo_claw, long baudrate)
{
    this->roboclaw = robo_claw;
    this->baudrate = baudrate;
    this->roboclaw->begin(baudrate);
}

void Robot::moveRobot(geometry_msgs__msg__Twist *cmdvel_msg, unsigned long *prev_cmd_time){
    if((millis()-*prev_cmd_time)>=200){
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