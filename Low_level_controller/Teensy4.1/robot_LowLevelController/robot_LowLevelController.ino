//micro ros headers
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

//message type headers
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>

//custom headers
//#include "imuBNO005.h"
//#include "RoboClaw.h"
#include "robot.h"

//defines
#define LED_PIN 13
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

//Objects for ros node, publisher and subscriber.
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_publisher_t odom_publisher, imu_publisher;
rcl_subscription_t cmdvel_subscriber;
bool micro_ros_init_successful;

//variables for ros msgs
sensor_msgs__msg__Imu imuMsg;
geometry_msgs__msg__Twist cmdvel_msg;
nav_msgs__msg__Odometry odom_msg;

//time variables
unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
const unsigned int timer_timeout = 100;

//Robot object
Robot robot(0.15, 0.525, 150); // Wheel radius , track width, max rpm

// Imu object
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire2); // PIN 25 to IMU SDA PIN , PIN 24 to IMU SCL PIN 

//Motor driver object
RoboClaw roboclaw = RoboClaw(&Serial2, 10000); //PIN 8 TX2 to roboclaw S1 Signal pin and PIN 7 RX2 to roboclaw S2 Signal pin

//Encoder objects
QuadEncoder leftEncoder(1,0,1,0);   // Encoder on channel 1 of 4 available, 
                                    // Phase A (pin0), PhaseB(pin1), Pullups Req(0)
QuadEncoder rightEncoder(2,2,3,0);   // Encoder on channel 1 of 4 available, 
                                    // Phase A (pin2), PhaseB(pin3), Pullups Req(0)

//connection states for microcontroller and micro ros agent on the PC
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

//callback for the timer assiged to publisher
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void) last_call_time;
  if (timer != NULL) {
    
    struct timespec time_stamp = getTime();

    //Updating the time stamps for the publishing msgs
    odom_msg.header.stamp.sec = time_stamp.tv_sec;
    odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;
    
    imuMsg.header.stamp.sec = time_stamp.tv_sec;
    imuMsg.header.stamp.nanosec = time_stamp.tv_nsec;

    //Update the IMU data and odometry data 
    robot.updateImuBNO055Data(&bno, &imuMsg);
    robot.updateOdometryData();

    //Publish the Imu and odom msgs
    rcl_publish(&imu_publisher, &imuMsg, NULL);
    rcl_publish(&odom_publisher, &odom_msg, NULL);
  }
}

// subscriber call back function
void cmdvel_sub_callback(const void *msgin){
  prev_cmd_time = millis();

  //Move the robot according to the input cmd_vel msg
  robot.moveRobot(&cmdvel_msg, prev_cmd_time);
}

//If the connection is established the entities are destroied

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "ulrich_robot_node", "", &support));

  // create IMU publisher
  RCCHECK(rclc_publisher_init_default(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu/data")
  );

  // create odom publisher
  RCCHECK(rclc_publisher_init_default(
    &odom_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "/odom")
  );

  //create twist msg subscriber
  RCCHECK(rclc_subscription_init_default( 
    &cmdvel_subscriber, 
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel")
  );

  // create timer, it controlles the publish rate
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator)); //2 handles one for timer and one for subscriber
  
  //adding timer to the executor
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  //adding subscriber to the executor
  RCCHECK(rclc_executor_add_subscription(&executor, &cmdvel_subscriber, &cmdvel_msg, &cmdvel_sub_callback, ON_NEW_DATA));

  // synchronize time with the agent
  syncTime();

  return true;
}

//If the connection is broke the entities are destroied
void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&imu_publisher, &node);
  rcl_publisher_fini(&odom_publisher, &node); 
  rcl_subscription_fini(&cmdvel_subscriber, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void syncTime()
{
  // get the current time from the agent
  unsigned long now = millis();
  RCCHECK(rmw_uros_sync_session(10));
  unsigned long long ros_time_ms = rmw_uros_epoch_millis(); 
  // now we can find the difference between ROS time and uC time
  time_offset = ros_time_ms - now;
}

struct timespec getTime()
{
  struct timespec tp = {0};
  // add time difference between uC time and ROS time to
  // synchronize time with ROS
  unsigned long long now = millis() + time_offset;
  tp.tv_sec = now / 1000;
  tp.tv_nsec = (now % 1000) * 1000000;
  
  return tp;
}

void setup() {
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);

  state = WAITING_AGENT;

  robot.EncodersInit(&leftEncoder, &rightEncoder, 20480); //encoder variable address and resolution of the encoders
  robot.OdometryInit(&odom_msg); //odom msg variable address and the update time to calculate the velocity
  robot.motorsInit(&roboclaw, 115200);    //roboclaw object address and baudrate 
  robot.imuBNO055_init(&bno);
}


//create and destroy the entities with respect to the connection state
void loop() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }

  if (state == AGENT_CONNECTED) {
    digitalWrite(LED_PIN, 1);
  } else {
    digitalWrite(LED_PIN, 0);
  }
}
