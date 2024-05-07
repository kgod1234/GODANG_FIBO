#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <stdio.h>
#include <iostream>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/string.h>

#include <Motor.h>
#include <Kinematics.h>
#include <pio_encoder.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rcl_subscription_t subscriber;
rcl_publisher_t publisher_log;
std_msgs__msg__String msg_log;
std_msgs__msg__Float32MultiArray msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

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

// Motor Setup
bool led_state = LOW;
int period = 2400;
float deltaT = period / 1.0e6;
Motor FL(8, 20, 21, 17000, 100, 100);
Motor FR(6, 10, 11, 17000, 100, 100);
Motor BL(4, 14, 15, 17000, 100, 100);
Motor BR(2, 12, 13, 17000, 100, 100);
// Edit Here

// Edit Here
float wheelDiameter = 0.127;
float lx = 0.26;
float ly = 0.432;
Kinematics kinematics(wheelDiameter, lx, ly);
// Edit Here

// variable
long start_time, T;
int currentStep = 0;
int stepsCount = 0;
bool newStepsAvailable = false;

float vx = 0;
float vy = 0;
float wz = 0;

// Transform
struct TransformStep
{
	float vx;
	float vy;
	float wz;
	unsigned long duration;
};

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

	vx = msg->data.data[1];
	vy = msg->data.data[0];
	wz = msg->data.data[2];

}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL)
	{

		String log_data = String(vx) + "," + String(vy) + "," + String(wz); 
		RCSOFTCHECK(rcl_publish(&publisher_log, &msg, NULL));

  		Kinematics::RadPS wheelSpeeds = kinematics.Inverse_Kinematics(vx, vy, wz);
  		FL.compute(wheelSpeeds.radps_fl, deltaT);
  		FR.compute(wheelSpeeds.radps_fr, deltaT);
  		BL.compute(wheelSpeeds.radps_bl, deltaT);
  		BR.compute(wheelSpeeds.radps_br, deltaT);
	}
}

void setup()
{
	Serial.begin(115200);
	set_microros_serial_transports(Serial);
	delay(100);

	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, HIGH);

	delay(2000);

	FR.encoder.begin();
	FL.encoder.begin();
	BR.encoder.begin();
	BL.encoder.begin();

	allocator = rcl_get_default_allocator();

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	RCCHECK(rclc_node_init_default(&node, "micro_ros_mobile_node", "", &support));

	// create subscriber
	RCCHECK(rclc_subscription_init_default(
		&subscriber, 
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
		"joy_data"));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher_log,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
		"logger"));

	// create timer
	const unsigned int timer_timeout = period;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	msg.data.capacity = 4;
	msg.data.size = 0;
	msg.data.data = (float_t*)malloc(msg.data.capacity * sizeof(float_t));

	msg.layout.dim.capacity = 4;
	msg.layout.dim.size = 0;
	msg.layout.dim.data =
			(std_msgs__msg__MultiArrayDimension*)malloc(msg.layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));

	for (size_t i = 0; i < msg.layout.dim.capacity; i++)
	{
		msg.layout.dim.data[i].label.capacity = 4;
		msg.layout.dim.data[i].label.size = 0;
		msg.layout.dim.data[i].label.data = (char*)malloc(msg.layout.dim.data[i].label.capacity * sizeof(char));
	}

	// create executor
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop()
{
	delay(100);
	RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1000)));
}