#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <stdio.h>
#include <iostream>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>

#include "SeedHarvester.h"
#include "BallShooter.h"

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

#define LED_PIN 25

float command = 0.0;
int block = 0;
int ballShooterState = 0;
bool maniState = true;
int maniStateCount = 0;

// SeedHarvester pins
#define grabberPin 26
#define groundlimit 27
#define toplimit 28
#define lifterA 14
#define lifterB 15
#define setzeroPin 4
#define dirPin_Seed 17
#define stepPin_Seed 16

SeedHarvester seedHarvester(grabberPin, lifterA, lifterB, groundlimit, toplimit, dirPin_Seed, stepPin_Seed, setzeroPin);

// BallShooter pins
#define servoPin 2
#define limitSwitchPin 3
#define INA 12
#define INB 13
#define grb_ref1 10
#define grb_ref2 11
#define stepPin_Ball 18
#define dirPin_Ball 19

BallShooter ballShooter(servoPin, limitSwitchPin, INA, INB, grb_ref1, grb_ref2, stepPin_Ball, dirPin_Ball);

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

void subscription_callback(const void* msgin)
{
  const std_msgs__msg__Float32MultiArray* msg = (const std_msgs__msg__Float32MultiArray*)msgin;
  digitalWrite(LED_PIN, HIGH);

  command = msg->data.data[3];

  if (command != 0 && command != 5.0 && maniState == true)
  {
    maniState = false;
  }
  if (maniStateCount == 10)
  {
    maniStateCount = 0;
    maniState = true;
  }

  if (command == 1.0 && ballShooterState != 1 && maniState == true)
  {
    // x button
    ballShooter.preparing();
    ballShooterState = 1;
  }
  else if (command == 2.0 && ballShooterState != 2 && maniState == true)
  {
    // o button
    ballShooter.grab();
    ballShooterState = 2;
  }
  else if (command == 18.0 && ballShooterState != 3 && maniState == true && ballShooterState == 2)
  {
    // trig left
    ballShooter.shoot();
    delay(100);
    ballShooter.preparing();
    ballShooterState = 3;
  }
  else if (command == 7.0 && maniState == true)
  {
    // trig right
    seedHarvester.single_press(true);
  }
  else if (command == 6.0 && maniState == true)
  {
    // up right
    seedHarvester.single_press(false);  // true == out
  }

  else if (command == 3.0 && maniState == true)
  {
    // triangle button
    seedHarvester.preparing();
  }
  else if (command == 4.0 && maniState == true)
  {
    // squre button
    seedHarvester.grab();
  }

  maniStateCount++;
}

void timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    debug_msg.data.data[3] = command;
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

  delay(2000);

  // setup is right hear
  ballShooter.setup();
  seedHarvester.setup();

  delay(5000);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_mani_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(&subscriber, &node,
                                         ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), "joy_data"));

  // create publisher
  RCCHECK(rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
                                      "debugging_mani"));

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