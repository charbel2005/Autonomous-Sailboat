#include "main.h"
#include "sensorGPS.h"

TaskHandle_t task_sensorGPS;

/**
  * Initialize the hardware.
  */
void sensorGPS_hardwareInit() {}

/**
  * Handler for the task.
  */
void sensorGPS_handler(void *argument) {
    for(;;) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for demonstration purposes
    }
}