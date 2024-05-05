#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

void setup()
{
  Serial.begin(921600);
}

void setup1()
{

}

void loop()
{
  Serial.println("loop");
  delay(100);
}

void loop1()
{
  Serial.println("loop1");
  delay(1000);
}