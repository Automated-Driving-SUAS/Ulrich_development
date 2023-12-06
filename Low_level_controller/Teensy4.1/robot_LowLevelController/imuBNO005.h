#ifndef _IMUBNO055_H_
#define _IMUBNO055_H_

//Arduino headers
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <sensor_msgs/msg/imu.h>

void imuBNO055_init(Adafruit_BNO055* bno);
void getImuBNO055Data(Adafruit_BNO055* bno, sensor_msgs__msg__Imu* imuMsg);


#endif 