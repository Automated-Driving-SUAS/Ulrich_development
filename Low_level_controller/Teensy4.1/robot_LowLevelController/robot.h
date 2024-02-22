#ifndef _ROBOT_H_
#define _ROBOT_H_

//IMU headers
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

//IMU functional headers
#include <Wire.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

// teensy 4.1 specific encoder header
#include "QuadEncoder.h"

//Roboclaw motor controler headers
#include "RoboClaw.h"

//micro-ros headers
#include <micro_ros_arduino.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

//message type headers
#include <sensor_msgs/msg/imu.h>
#include <nav_msgs/msg/odometry.h>


class Robot
{
private:
    double wheelRadius = 0.0, trackWidth = 0.0, xPos = 0.0, yPos = 0.0, theta = 0.0,
            maxRPM = 0.0, maxVel = 0.0;
    struct Vel
    {
        double left = 0.0;
        double right = 0.0;
    };
    Vel velActual;
    double encoderRes = 0, updateTime = 0.0;
    unsigned long prev_update_time = 0.0;
    RoboClaw *roboclaw;
    uint16_t baudrate = 38400;
    QuadEncoder *leftEncoder , *rightEncoder;
    nav_msgs__msg__Odometry *odom_msg;
private:
    const void euler_to_quat(float roll, float pitch, float yaw, double *q);
    void getRobotVelocity();
public:
    Vel velReq;
public:
    Robot(double wheelRadius, double trackWidth, double maxRPM);
    void EncodersInit(QuadEncoder *leftencoder, QuadEncoder *rightencoder, double encoderRes);
    void OdometryInit(nav_msgs__msg__Odometry *odom_msg);
    void updateOdometryData();
    void motorsInit(RoboClaw *roboclaw, long baudrate);
    void moveRobot(geometry_msgs__msg__Twist *cmdvel_msg, unsigned long *prev_cmd_time);

    void imuBNO055_init(Adafruit_BNO055* bno);
    void updateImuBNO055Data(Adafruit_BNO055* bno, sensor_msgs__msg__Imu* imuMsg);
};



#endif