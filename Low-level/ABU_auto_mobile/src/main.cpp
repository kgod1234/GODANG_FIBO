#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <stdio.h>
#include <iostream>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>

#include "Motor.h"
#include "Kinematics.h"
#include <RPi_Pico_TimerInterrupt.h>
#include <pio_encoder.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rcl_subscription_t subscriber;
rcl_publisher_t publisher;
std_msgs__msg__String pos_data;
std_msgs__msg__String vel_data;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Timer Interrupt Setup
#define dt_us 1000  // 1000 Hz
float deltaT = dt_us / 1.0e6;
RPI_PICO_Timer Timer(0);
bool TimerStatus = false;

// Motor Setup
Motor FL(2, 10, 11, 200, 20, 0);
Motor FR(4, 12, 13, 200, 20, 0);
Motor BL(6, 14, 15, 200, 20, 0);
Motor BR(8, 20, 21, 200, 20, 0);

float wheelDiameter = 0.127;
float lx = 0.388 / 2;
float ly = 0.375 / 2;
Kinematics kinematics(wheelDiameter, lx, ly);

Kinematics::Position currentPosition{ 0.0, 0.0, 0.0 };

String vx, vy, wz;

float radps_fl, radps_fr, radps_bl, radps_br;

struct TransformStep
{
  float vx;
  float vy;
  float wz;
  unsigned long duration;
};

// variable
long start_time, T;
int currentStep = 0;
int stepsCount = 0;
bool newStepsAvailable = false;
TransformStep steps[10];

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

void error_loop()
{
  while (1)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(2000);
  }
}

float strToFloat(String str)
{
  float result = 0.0;
  float decimalMultiplier = 1.0;
  bool isNegative = false;
  bool decimalPart = false;
  int length = str.length();

  if (str[0] == '-')
  {
    isNegative = true;
    str = str.substring(1);
    length--;
  }

  for (int i = 0; i < length; i++)
  {
    char c = str[i];

    if (c == '.')
    {
      decimalPart = true;
      continue;
    }

    if (decimalPart)
    {
      decimalMultiplier *= 0.1;
      result += (c - '0') * decimalMultiplier;
    }
    else
    {
      result = result * 10.0 + (c - '0');
    }
  }

  if (isNegative)
  {
    result = -result;
  }

  return result;
}

bool TimerHandler(struct repeating_timer* t) {
  (void)t;

  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    if (data == "Reset") {
      currentPosition = { 0.0, 0.0, 0.0 };  // Reset position
      vx = "0";
      vy = "0";
      wz = "0";
      // Serial.println("Position reset to 0,0,0");
    } else {
      int firstSemiColon = data.indexOf(';');
      int secondSemiColon = data.indexOf(';', firstSemiColon + 1);

      vx = strToFloat(data.substring(0, firstSemiColon));
      vy = strToFloat(data.substring(firstSemiColon + 1, secondSemiColon));
      wz = strToFloat(data.substring(secondSemiColon + 1));
    }
  }

  Kinematics::RadPS wheelSpeeds = kinematics.Inverse_Kinematics(strToFloat(vx), strToFloat(vy), strToFloat(wz));
  radps_fl = FL.computeRADS(wheelSpeeds.radps_fl, deltaT);
  radps_fr = FR.computeRADS(wheelSpeeds.radps_fr, deltaT);
  radps_bl = BL.computeRADS(wheelSpeeds.radps_bl, deltaT);
  radps_br = BR.computeRADS(wheelSpeeds.radps_br, deltaT);
  currentPosition = kinematics.Forward_Kinematics_Position(radps_fl, radps_fr, radps_bl, radps_br, currentPosition, deltaT);
}

void subscription_callback(const void* msgin)
{
  
}

void timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    RCSOFTCHECK(rcl_publish(&publisher, &pos_data, NULL));
  }
}

void setup()
{
  analogWriteFreq(1000);
  analogWriteRange(255);

  FR.encoder.begin();
  FL.encoder.begin();
  BR.encoder.begin();
  BL.encoder.begin();

  // Timer Interrupt Setup
  TimerStatus = Timer.attachInterruptInterval(dt_us, TimerHandler);
}

void setup1()
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
  RCCHECK(rclc_node_init_default(&node, "micro_ros_auto_mobile_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(&subscriber, &node,
                                         ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "vel_data"));

  // create publisher
  RCCHECK(rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                                      "pos_data"));

  const unsigned int timer_timeout = 20;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &vel_data, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer))

}

void loop()
{
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
}
