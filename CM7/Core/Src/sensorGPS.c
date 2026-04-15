#include "main.h"
#include "stm32h7xx_hal_i2c.h"
#include "sensorGPS.h"

#define M9N_ADDR 0x42  // default I2C address for the M9N GPS module, according to AI, never found it in the data sheet

TaskHandle_t task_sensorGPS;
I2C_HandleTypeDef I2C_M9N_Handle;

/**
  * Initialize the hardware.
  */
void sensorGPS_hardwareInit() {
  // Page 71 of the chip datasheet says pf15 and pf14 are I2c_SDA and I2c_SCL
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // using PF15 for I2C4_SDA
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD; // Open Drain - OD
  GPIO_InitStruct.Pull = GPIO_NOPULL; 
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C4;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  // using PF14 for I2C4_SCL
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  I2C_M9N_Handle.Instance = I2C4;
  // Fast mode (400kHz) timing from table ?? Dunno, AI made this, I found the 8MHz table on page 2135 of the refrence manual
  // PRESC=15 -> 64MHz/16 = 4MHz (250ns per tick)
  // Period = 1us + 1.5us = 2.5us -> 400kHz
  I2C_M9N_Handle.Init.Timing =
        (0xFU << 28) |  // PRESC  = 15 : 64MHz / 16 = 4MHz (250ns per tick)
        (0x0U << 24) |  // reserved
        (0x1U << 20) |  // SCLDEL =  1 : SCL data setup  = 2  ticks = 500ns
        (0x0U << 16) |  // SDADEL =  0 : SDA data hold   = 0  ticks = 0ns
        (0x03U << 8) |  // SCLH   =  3 : SCL high period = 4  ticks = 1us
        (0x05U << 0);   // SCLL   =  5 : SCL low  period = 6  ticks = 1.5us

    I2C_M9N_Handle.Init.AddressingMode =  I2C_ADDRESSINGMODE_7BIT;
    I2C_M9N_Handle.Init.OwnAddress1 = 0;
    I2C_M9N_Handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    I2C_M9N_Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    I2C_M9N_Handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    
    if (HAL_I2C_Init(&I2C_M9N_Handle) != HAL_OK) {
        printf("I2C Init Error");
        Error_Handler();
    }
}

/**
  * Handler for the task.
  */
void sensorGPS_handler(void *argument) {
    for(;;) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for demonstration purposes
    }
}