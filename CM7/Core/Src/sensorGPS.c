

// #include "cmsis_os2.h"
#include "main.h"
// #include "servoSail.h"

#define TASK_NAME "magnetometerTask"
#define TASK_STACK_SIZE 128
#define TASK_PRIORITY osPriorityAboveNormal
#define BNO055_ADDR  0x29  // default, COM3 high

TaskHandle_t task_sensorGPS

void magnetometer_handle(void *argument);
I2C_HandleTypeDef I2C_BNO055_Handle;

/**
  * Initialize the hardware.
  */
void magnometer_hardwareInit()
{
    // Page 65 of the chip datasheet says pf0 and pf1 are I2c_SDA and I2c_SCL
    // added  __HAL_RCC_GPIOF_CLK_ENABLE(); to the main.c
    // added __HAL_RCC_I2C2_CLK_ENABLE(); to the main.c
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD; // Open Drain - OD
  GPIO_InitStruct.Pull = GPIO_NOPULL; 
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_1;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  // HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *hi2c);                      line 601 of Stm32h7xx_hal_i2c.h
  // I2C_TypeDef                *Instance;      /*!< I2C registers base address    line 186 of Stm32h7xx_hal_i2c.h
  // I2C_InitTypeDef            Init;           /*!< I2C communication parameters  line 187 of Stm32h7xx_hal_i2c.h
    I2C_BNO055_Handle.Instance = I2C2; 
    // The board's clock rate is 64MHz accoding to servoSail.c

    // 0x60201E28 for 100khz I2C signal

    // PRESC = 6: Divides 64MHz down to a manageable internal frequency.

    // SCLH/SCLL: Set to create a symmetrical 100kHz square wave.
    I2C_BNO055_Handle.Init.Timing = 0x60201E28;
    I2C_BNO055_Handle.Init.I2C_ADDRESSINGMODE_7BIT
    I2C_BNO055_Handle.Init.OwnAddress1 = 0;
    I2C_BNO055_Handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    I2C_BNO055_Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    I2C_BNO055_Handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&I2C_BNO055_Handle);
    // then we need to do the write transaciton
    uint8_t reg = 0x0F;

    HAL_I2C_Master_Transmit(&I2C_BNO055_Handle, BNO055_ADDR << 1, &reg, 1, HAL_MAX_DELAY);
    // we do 1 cause we want to write data to there
}

// 0xAA is the start byte
