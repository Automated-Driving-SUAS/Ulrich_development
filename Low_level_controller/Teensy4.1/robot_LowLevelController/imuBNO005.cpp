#include "imuBNO005.h"


void imuBNO055_init(Adafruit_BNO055* bno){
    bno->begin();
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


void getImuBNO055Data(Adafruit_BNO055* bno, sensor_msgs__msg__Imu* imuMsg){

    sensors_event_t angVelocityData, linearAccelData;
    bno->getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno->getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Quaternion quat = bno->getQuat();
  
    imuMsg->header.frame_id = micro_ros_string_utilities_set(imuMsg->header.frame_id, "imu_link");

    imuMsg->linear_acceleration.x = linearAccelData.acceleration.x;
    imuMsg->linear_acceleration.y = linearAccelData.acceleration.x;
    imuMsg->linear_acceleration.z = linearAccelData.acceleration.x;

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