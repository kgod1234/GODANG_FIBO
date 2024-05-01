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

struct MobileData
{
	float vx = 0.0f;
	float vy = 0.0f;
	float wz = 0.0f;
};

#define MOTOR_FL_A 20
#define MOTOR_FL_B 21

#define MOTOR_FR_A 10
#define MOTOR_FR_B 11

#define MOTOR_BL_A 14
#define MOTOR_BL_B 15

#define MOTOR_BR_A 12
#define MOTOR_BR_B 13

MobileData mobile_data;

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

	mobile_data.vx = msg->data.data[0];
	mobile_data.vy = msg->data.data[1];
	mobile_data.wz = msg->data.data[2];

	if (mobile_data.vx == 0)
	{
		mobile_data.vx = 0;
	}
	else if (mobile_data.vx < 0)
	{
		mobile_data.vx = round(mapfloat(mobile_data.vx, -1, 0, -255, 0));
	}
	else
	{
		mobile_data.vx =  round(mapfloat(mobile_data.vx, 0, 1, 0, 255));
	}

	if (mobile_data.vy == 0)
	{
		mobile_data.vy = 0;
	}
	else if (mobile_data.vy < 0)
	{
		mobile_data.vy = round(mapfloat(mobile_data.vy, -1, 0, -255, 0));
	}
	else
	{
		mobile_data.vy = round(mapfloat(mobile_data.vy, 0, 1, 0, 255));
	}

	if (mobile_data.wz == 0)
	{
		mobile_data.wz = 0;
	}
	else if (mobile_data.wz < 0)
	{
		mobile_data.wz = round(mapfloat(mobile_data.wz, -1, 0, -255, 0));
	}
	else
	{
		mobile_data.wz =  round(mapfloat(mobile_data.wz, 0, 1, 0, 255));
	}
}

void timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL)
	{
		debug_msg.data.data[1] = mobile_data.vx;
		debug_msg.data.data[0] = mobile_data.vy;
		debug_msg.data.data[2] = mobile_data.wz;

		RCSOFTCHECK(rcl_publish(&publisher, &debug_msg, NULL));

		if (mobile_data.vy > 0)
		{
			analogWrite(MOTOR_FL_A, mobile_data.vy);
			analogWrite(MOTOR_FL_B, 0);
			analogWrite(MOTOR_FR_A, mobile_data.vy);
			analogWrite(MOTOR_FR_B, 0);

			analogWrite(MOTOR_BL_A, mobile_data.vy);
			analogWrite(MOTOR_BL_B, 0);
			analogWrite(MOTOR_BR_A, mobile_data.vy);
			analogWrite(MOTOR_BR_B, 0);
		}
		else if (mobile_data.vy < 0)
		{
			analogWrite(MOTOR_FL_A, 0);
			analogWrite(MOTOR_FL_B, -mobile_data.vy);
			analogWrite(MOTOR_FR_A, 0);
			analogWrite(MOTOR_FR_B, -mobile_data.vy);

			analogWrite(MOTOR_BL_A, 0);
			analogWrite(MOTOR_BL_B, -mobile_data.vy);
			analogWrite(MOTOR_BR_A, 0);
			analogWrite(MOTOR_BR_B, -mobile_data.vy);
			
		}
		else if (mobile_data.vx > 0)
		{
			analogWrite(MOTOR_FL_A, mobile_data.vx);
			analogWrite(MOTOR_FL_B, 0);
			analogWrite(MOTOR_FR_A, 0);
			analogWrite(MOTOR_FR_B, mobile_data.vx);

			analogWrite(MOTOR_BL_A, 0);
			analogWrite(MOTOR_BL_B, mobile_data.vx);
			analogWrite(MOTOR_BR_A, mobile_data.vx);
			analogWrite(MOTOR_BR_B, 0);
		}
		else if (mobile_data.vx < 0)
		{
			analogWrite(MOTOR_FL_A, 0);
			analogWrite(MOTOR_FL_B, -mobile_data.vx);
			analogWrite(MOTOR_FR_A, -mobile_data.vx);
			analogWrite(MOTOR_FR_B, 0);

			analogWrite(MOTOR_BL_A, -mobile_data.vx);
			analogWrite(MOTOR_BL_B, 0);
			analogWrite(MOTOR_BR_A, 0);
			analogWrite(MOTOR_BR_B, -mobile_data.vx);
		}
		else if (mobile_data.wz > 0)
		{
			analogWrite(MOTOR_FL_A, mobile_data.wz);
			analogWrite(MOTOR_FL_B, 0);
			analogWrite(MOTOR_FR_A, 0);
			analogWrite(MOTOR_FR_B, mobile_data.wz);

			analogWrite(MOTOR_BL_A, mobile_data.wz);
			analogWrite(MOTOR_BL_B, 0);
			analogWrite(MOTOR_BR_A, 0);
			analogWrite(MOTOR_BR_B, mobile_data.wz);
		}
		else if (mobile_data.wz < 0)
		{
			analogWrite(MOTOR_FL_A, 0);
			analogWrite(MOTOR_FL_B, -mobile_data.wz);
			analogWrite(MOTOR_FR_A, -mobile_data.wz);
			analogWrite(MOTOR_FR_B, 0);

			analogWrite(MOTOR_BL_A, 0);
			analogWrite(MOTOR_BL_B, -mobile_data.wz);
			analogWrite(MOTOR_BR_A, -mobile_data.wz);
			analogWrite(MOTOR_BR_B, 0);
		}
		else
		{
			analogWrite(MOTOR_FL_A, 0);
			analogWrite(MOTOR_FL_B, 0);
			analogWrite(MOTOR_FR_A, 0);
			analogWrite(MOTOR_FR_B, 0);

			analogWrite(MOTOR_BL_A, 0);
			analogWrite(MOTOR_BL_B, 0);
			analogWrite(MOTOR_BR_A, 0);
			analogWrite(MOTOR_BR_B, 0);
		}
	}
}

void setup()
{
	Serial.begin(921600);
	set_microros_serial_transports(Serial);
	delay(100);

	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, HIGH);

	pinMode(MOTOR_FL_A, OUTPUT);
	pinMode(MOTOR_FL_B, OUTPUT);
	pinMode(MOTOR_FR_A, OUTPUT);
	pinMode(MOTOR_FR_B, OUTPUT);

	pinMode(MOTOR_BL_A, OUTPUT);
	pinMode(MOTOR_BL_B, OUTPUT);
	pinMode(MOTOR_BR_A, OUTPUT);
	pinMode(MOTOR_BR_B, OUTPUT);

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