#ifndef _ROBOT_H_
#define _ROBOT_H_

//micro-ros headers

#include <micro_ros_arduino.h>
#include <micro_ros_utilities/string_utilities.h>
#include <nav_msgs/msg/odometry.h>

// teensy 4.1 specific encoder header
#include "QuadEncoder.h"
#include "RoboClaw.h"


class Robot
{
private:
    float wheelRadius = 0.0, trackWidth = 0.0, xPos = 0.0, yPos = 0.0, theta = 0.0,
            maxRPM = 0.0, maxVel = 0.0;
    struct Vel
    {
        float left = 0.0;
        float right = 0.0;
    };
    Vel velActual;
    unsigned int encoderRes = 0, updateTime = 0.0;
    RoboClaw *roboclaw;
    long baudrate = 38400;
    QuadEncoder *leftEncoder , *rightEncoder;
    nav_msgs__msg__Odometry *odom_msg;
private:
    const void euler_to_quat(float roll, float pitch, float yaw, double *q);
    void getRobotVelocity();
public:
    Vel velReq;
public:
    Robot(float wheelRadius, float trackWidth , float maxRPM );
    void EncodersInit(QuadEncoder *leftencoder, QuadEncoder *rightencoder, unsigned int encoderRes);
    void OdometryInit(nav_msgs__msg__Odometry *odom_msg, unsigned int updateTime);
    void updateOdometry();
    void motorsInit(RoboClaw *robo_claw, long baudrate);
    void moveRobot(geometry_msgs__msg__Twist *cmdvel_msg, unsigned long *prev_cmd_time);
};



#endif