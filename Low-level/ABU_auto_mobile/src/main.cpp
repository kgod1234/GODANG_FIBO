#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>

// Task function for Core 0
void taskCore0(void* parameter)
{
  for (;;)
  {
    Serial.println("Task running on Core 0");
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

// Task function for Core 1
void taskCore1(void* parameter)
{
  for (;;)
  {
    Serial.println("Task running on Core 1");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  Serial.begin(9600);

  // Create tasks on each core
  xTaskCreate(taskCore0, "TaskCore0", 128, NULL, 1, NULL);  // Core 0
  xTaskCreate(taskCore1, "TaskCore1", 128, NULL, 1, NULL);  // Core 1
}

void loop()
{
  // Code in loop() will not be executed in this configuration
  // because tasks are managed by FreeRTOS
}