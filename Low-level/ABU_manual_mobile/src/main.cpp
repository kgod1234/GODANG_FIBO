#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <stdio.h>
#include <iostream>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>

#include <Motor.h>
#include <Kinematics.h>
#include <pio_encoder.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rcl_subscription_t subscriber;
rcl_publisher_t publisher;
std_msgs__msg__Float32MultiArray debug_msg;
std_msgs__msg__Float32MultiArray msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Edit Here
// Motor Setup
Motor FL(8, 14, 15, 17000, 100, 100);
Motor FR(6, 10, 11, 17000, 100, 100);
Motor BL(4, 20, 21, 17000, 100, 100);
Motor BR(2, 12, 13, 17000, 100, 100);
// Edit Here

float wheelDiameter = 0.127;
float lx = 0.26;
float ly = 0.432;
Kinematics kinematics(wheelDiameter, lx, ly);

// For test purpose
// variable
long start_time, T;
int currentStep = 0;
int stepsCount = 0;
bool newStepsAvailable = false;

bool slowState = false;

// Transform
struct TransformStep
{
  float vx;
  float vy;
  float wz;
  unsigned long duration;
};

TransformStep mobile_data;

#define LED_PIN 25

#define RCCHECK(fn)                                                                                                    \
  {                                                                                                                    \
    rcl_ret_t temp_rc = fn;                                                                                            \
    if ((temp_rc != RCL_RET_OK))                                                                                       \
    {                                                                                                                  \
      error_loop();                                                                                                    \
    }                                                                                                                  \
  }
#define RCSOFTCHECK(fn)                                                                                                \
  {                                                                                                                    \
    rcl_ret_t temp_rc = fn;                                                                                            \
    if ((temp_rc != RCL_RET_OK))                                                                                       \
    {                                                                                                                  \
    }                                                                                                                  \
  }
// control setup

void error_loop()
{
  while (1)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(2000);
  }
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

void subscription_callback(const void* msgin)
{
  const std_msgs__msg__Float32MultiArray* msg = (const std_msgs__msg__Float32MultiArray*)msgin;
  digitalWrite(LED_PIN, HIGH);

  mobile_data.vx = mapfloat(msg->data.data[0], -1.0f, 1.0f, -0.92f, 0.92f);
  mobile_data.vy = mapfloat(msg->data.data[1] * -1, -1.0f, 1.0f, -0.92f, 0.92f);
  mobile_data.wz = mapfloat(msg->data.data[2], -1.0f, 1.0f, -2.65f, 2.65f);

  if (msg->data.data[3] == 5)
  {
    slowState == true;
  }
  else
  {
    slowState == false;
  }
}

void timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    // debug_msg.data.data[1] = mobile_data.vx;
    // debug_msg.data.data[0] = mobile_data.vy;
    // debug_msg.data.data[2] = mobile_data.wz;

    if (slowState)
    {
      Kinematics::RPM wheelSpeeds = kinematics.Inverse_Kinematics(mobile_data.vx, mobile_data.vy,
                                                                  -1 * (mobile_data.wz));  // Set Joy to 0.92 0.92 2.65
      FL.setSpeed(mapfloat(wheelSpeeds.RPM_FL, -138, 138, -255, 255) / 2);
      FR.setSpeed(mapfloat(wheelSpeeds.RPM_FR, -138, 138, -255, 255) / 2);
      BL.setSpeed(mapfloat(wheelSpeeds.RPM_BL, -138, 138, -255, 255) / 2);
      BR.setSpeed(mapfloat(wheelSpeeds.RPM_BR, -138, 138, -255, 255) / 2);

      debug_msg.data.data[0] = mapfloat(wheelSpeeds.RPM_FL, -138, 138, -255, 255) / 2;
      debug_msg.data.data[1] = mapfloat(wheelSpeeds.RPM_FR, -138, 138, -255, 255) / 2;
      debug_msg.data.data[2] = mapfloat(wheelSpeeds.RPM_BL, -138, 138, -255, 255) / 2;
      debug_msg.data.data[3] = mapfloat(wheelSpeeds.RPM_BR, -138, 138, -255, 255) / 2;
    }
    else
    {
      Kinematics::RPM wheelSpeeds = kinematics.Inverse_Kinematics(mobile_data.vx, mobile_data.vy,
                                                                  -1 * (mobile_data.wz));  // Set Joy to 0.92 0.92 2.65
      FL.setSpeed(mapfloat(wheelSpeeds.RPM_FL, -138, 138, -255, 255));
      FR.setSpeed(mapfloat(wheelSpeeds.RPM_FR, -138, 138, -255, 255));
      BL.setSpeed(mapfloat(wheelSpeeds.RPM_BL, -138, 138, -255, 255));
      BR.setSpeed(mapfloat(wheelSpeeds.RPM_BR, -138, 138, -255, 255));

      debug_msg.data.data[0] = mapfloat(wheelSpeeds.RPM_FL, -138, 138, -255, 255);
      debug_msg.data.data[1] = mapfloat(wheelSpeeds.RPM_FR, -138, 138, -255, 255);
      debug_msg.data.data[2] = mapfloat(wheelSpeeds.RPM_BL, -138, 138, -255, 255);
      debug_msg.data.data[3] = mapfloat(wheelSpeeds.RPM_BR, -138, 138, -255, 255);
    }

    RCSOFTCHECK(rcl_publish(&publisher, &debug_msg, NULL));
  }
}

void setup()
{
  Serial.begin(921600);
  set_microros_serial_transports(Serial);
  delay(100);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  FR.encoder.begin();
  FL.encoder.begin();
  BR.encoder.begin();
  BL.encoder.begin();

  delay(2000);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_mobile_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(&subscriber, &node,
                                         ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "joy_data"));

  // create publisher
  RCCHECK(rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
                                      "debugging"));

  const unsigned int timer_timeout = 20;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

  msg.data.capacity = 4;
  msg.data.size = 4;
  msg.data.data = (float_t*)malloc(msg.data.capacity * sizeof(float_t));

  msg.layout.dim.capacity = 4;
  msg.layout.dim.size = 4;
  msg.layout.dim.data =
      (std_msgs__msg__MultiArrayDimension*)malloc(msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));

  for (size_t i = 0; i < msg.layout.dim.capacity; i++)
  {
    msg.layout.dim.data[i].label.capacity = 4;
    msg.layout.dim.data[i].label.size = 4;
    msg.layout.dim.data[i].label.data = (char*)malloc(msg.layout.dim.data[i].label.capacity * sizeof(char));
  }

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer))

  debug_msg.data.capacity = 4;
  debug_msg.data.size = 4;
  debug_msg.data.data = (float_t*)malloc(msg.data.capacity * sizeof(float_t));

  msg.data.data[0] = 0.0f;
  msg.data.data[1] = 0.0f;
  msg.data.data[2] = 0.0f;
  msg.data.data[3] = 0.0f;
}

void loop()
{
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
}