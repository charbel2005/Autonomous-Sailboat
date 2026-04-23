#include "main.h"
#include "stm32h7xx_hal_i2c.h"
#include <stdint.h>
#include "sensorMagnetometer.h"

#define BNO055_ADDR  0x28  // ADR pin LOW (default Adafruit breakout). Use 0x29 if ADR pin is HIGH.
#define BNO055_WHO_AM_I 0x00  // chip ID register
#define BNO055_ACC 0x01  // chip ID register
#define BNO055_MAG 0x02  // chip ID register
#define BNO055_GYRO 0x03  // chip ID register
#define BNO055_ACCX_LSB 0x08  // chip ID register
#define BNO055_ACCX_MSB 0x09  // chip ID register
#define BNO055_ACCY_LSB 0x0A  // chip ID register
#define BNO055_ACCY_MSB 0x0B  // chip ID register
#define BNO055_ACCZ_LSB 0x0C  // chip ID register
#define BNO055_ACCZ_MSB 0x0D  // chip ID register
#define BNO055_MAGX_LSB 0x0E  // chip ID register
#define BNO055_MAGX_MSB 0x0F  // chip ID register
#define BNO055_MAGY_LSB 0x10  // chip ID register
#define BNO055_MAGY_MSB 0x11  // chip ID register
#define BNO055_MAGZ_LSB 0x12  // chip ID register
#define BNO055_MAGZ_MSB 0x13  // chip ID register
#define BNO055_GYRX_LSB 0x14  // chip ID register
#define BNO055_GYRX_MSB 0x15  // chip ID register
#define BNO055_GYRY_LSB 0x16  // chip ID register
#define BNO055_GYRY_MSB 0x17  // chip ID register
#define BNO055_GYRZ_LSB 0x18  // chip ID register
#define BNO055_GYRZ_MSB 0x19  // chip ID register
#define BNO055_EUL_HEADING_LSB 0x1A  // chip ID register
#define BNO055_EUL_HEADING_MSB 0x1B  // chip ID register
#define BNO055_ST_RESULT 0x36 // Self-test register
#define BNO055_SYS_STAT 0x39 // System status register
#define BNO055_SYS_ERROR 0x3A // System error register
#define BNO055_OPR_MODE 0x3D // Operation mode register
#define BNO55_CALIB_STATE 0x35 // Calibration staus register
#define BNO055_CALIB 0x55 // Lowest calibration register ADDR
#define CALIB_OFFSET_SIZE 22 // How many calibration regs to read
#define BNOSS_SICMATRIX 0x43 // Lowest calibration register indluding SIC MATRIX

#define BNO055_OPR_MODE_CONFIG 0x00
#define BNO055_OPR_MODE_ACCONLY 0x01
#define BNO055_OPR_MODE_MAGONLY 0x02
#define BNO055_OPR_MODE_GYROONLY 0x03
#define BNO055_OPR_MODE_ACCMAG 0x04
#define BNO055_OPR_MODE_ACCGYRO 0x05
#define BNO055_OPR_MODE_MAGGYRO 0x06
#define BNO055_OPR_MODE_AMG 0x07
#define BNO055_OPR_MODE_IMUPLUS 0x08
#define BNO055_OPR_MODE_COMPASS 0x09
#define BNO055_OPR_MODE_M4G 0x0A
#define BNO055_OPR_MODE_NDOF_FMC_OFF 0x0B
#define BNO055_OPR_MODE_NDOF 0x0C

#define TRUE 0x01
#define FALSE 0x00

void sensorMagnetometer_setOperationMode(uint8_t mode);
static void sensorMagnetometer_readChip(uint8_t regADDR, const char *name);
void sensorMagnetometer_readWhoAmI();
void sensorMagnetometer_readMAG();
void sensorMagnetometer_readACC();
void sensorMagnetometer_readGYRO();
void sensorMagnetometer_readSelfTest();
void sensorMagnetometer_readGYRO_Vector();
void sensorMagnetometer_readMAG_Vector();
void sensorMagnetometer_readACC_Vector();
void sensorMagnetometer_readVectorDynamic(uint8_t startReg, uint8_t bytes, const char *name);
uint8_t checkCalibration();

#pragma pack(1) // ensure no padding between fields
typedef struct {
    // Raw sensor vectors
    int16_t acc_x,  acc_y,  acc_z;
    int16_t gyro_x, gyro_y, gyro_z;
    int16_t mag_x,  mag_y,  mag_z;

    // Fusion Mode Values
    int16_t heading, pitch, roll;

    // Claude said to have ths for when we transfer between cores
    uint32_t update_count;
} IMU_Data;
#pragma pack()

IMU_Data IMU = {0};
TaskHandle_t task_sensorMagnetometer;
I2C_HandleTypeDef I2C_BNO055_Handle;
uint8_t currentMode;
static uint8_t  isCalibrated = FALSE;
static uint8_t  savedOffsets[CALIB_OFFSET_SIZE] = {0};

/**
  * Initialize the hardware.
  */
void sensorMagnetometer_hardwareInit()
{
    // Page 65 of the chip datasheet says pf0 and pf1 are I2c_SDA and I2c_SCL
    // added  __HAL_RCC_GPIOF_CLK_ENABLE(); to the main.c
    // added __HAL_RCC_I2C2_CLK_ENABLE(); to the main.c
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // using PB11 for I2C2_SDA
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD; // Open Drain - OD
    GPIO_InitStruct.Pull = GPIO_NOPULL; 
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // using PB10 for I2C2_SCL
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *hi2c);                      line 601 of Stm32h7xx_hal_i2c.h
    // I2C_TypeDef                *Instance;      /*!< I2C registers base address    line 186 of Stm32h7xx_hal_i2c.h
    // I2C_InitTypeDef            Init;           /*!< I2C communication parameters  line 187 of Stm32h7xx_hal_i2c.h
    I2C_BNO055_Handle.Instance = I2C2; 

    //<---------------Connor and Charbel Timing Setup----------------->
    // Use table example in reference manual (use 64Mhz base clock divide by 16 to get 4MHz)
    // For each frequency the tables prescales each frequency into 4Mhz and uses 4Mhz for every other settings
    // That's why prescale is 15 since 64MHz / 16 = 4MHz
    //
    // I2C TIMINGR register layout (RM0433 reference manual):
    // [31:28] PRESC  - Prescaler: divides I2C kernel clock. tick = 1/(f_i2cclk / (PRESC+1))
    // [27:24] (reserved, must be 0)
    // [23:20] SCLDEL - SCL data setup delay (in prescaled ticks)
    // [19:16] SDADEL - SDA data hold delay  (in prescaled ticks)
    // [15:8]  SCLH   - SCL high period      (in prescaled ticks, actual = SCLH+1)
    // [7:0]   SCLL   - SCL low period       (in prescaled ticks, actual = SCLL+1)
    //
    // With 64MHz kernel clock and PRESC=15: tick = 1/(64MHz/16) = 250ns
    // SCLDEL=4 -> setup  = 5   * 250ns = 1250ns
    // SDADEL=2 -> hold   = 2   * 250ns =  500ns
    // SCLH=15  -> high   = 16  * 250ns = 4000ns
    // SCLL=19  -> low    = 20  * 250ns = 5000ns
    // f_SCL = 1 / (4000ns + 5000ns) ~= 111kHz (standard-mode 100kHz, rise/fall times account for the rest)
    I2C_BNO055_Handle.Init.Timing =
        (0xFU << 28) |  // PRESC  = 15 : 64MHz / 16 = 4MHz (250ns per tick)
        (0x0U << 24) |  // reserved
        (0x4U << 20) |  // SCLDEL =  4 : SCL data setup  = 5   ticks = 1.25us
        (0x2U << 16) |  // SDADEL =  2 : SDA data hold   = 2   ticks = 500ns
        (0x0FU << 8) |  // SCLH   = 15 : SCL high period = 16  ticks = 4us
        (0x13U << 0);   // SCLL   = 19 : SCL low  period = 20  ticks = 5us

    I2C_BNO055_Handle.Init.AddressingMode =  I2C_ADDRESSINGMODE_7BIT;
    I2C_BNO055_Handle.Init.OwnAddress1 = 0x1;
    I2C_BNO055_Handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    I2C_BNO055_Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    I2C_BNO055_Handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    
    if (HAL_I2C_Init(&I2C_BNO055_Handle) != HAL_OK) {
        printf("I2C Init Error");
        Error_Handler();
    }

    // BNO055 requires up to 650ms after power-on before it responds to I2C.
    // Without this delay the first transaction gets a NACK (error 0x2) which
    // can then leave the peripheral in a stuck state (error 0x20).
    HAL_Delay(700);

    sensorMagnetometer_readWhoAmI();
    sensorMagnetometer_readSelfTest();
    currentMode = BNO055_OPR_MODE_NDOF; // USER: only change this line to set mode

    if (!(currentMode >= BNO055_OPR_MODE_IMUPLUS && currentMode <= BNO055_OPR_MODE_NDOF))
    {
        // Non-fusion mode — just set it, no calibration needed
        sensorMagnetometer_setOperationMode(currentMode);
    }
    else if (isCalibrated)
    {
        // Already calibrated from a previous boot — restore offsets then go to fusion mode
        // loadCalibrationOffsets internally switches to CONFIG_MODE to write safely
        BNO055_loadCalibrationOffsets();
        sensorMagnetometer_setOperationMode(currentMode);
    }
    else
    {
        // First boot in fusion mode — must set fusion mode FIRST so the calibration
        // algorithm runs, then poll until all three sensors reach 3/3
        sensorMagnetometer_setOperationMode(currentMode);
        HAL_Delay(20);

        printf("Move sensor in figure-8 for mag, hold 6 orientations for acc, keep still for gyro\r\n");

        while (BNO055_checkCalibration() == 0)
        {
            HAL_Delay(2000);
        }

        // saveCalibrationOffsets switches to CONFIG_MODE internally to read offsets
        BNO055_saveCalibrationOffsets();
        isCalibrated = TRUE;
        printf("BNO055 fully calibrated\r\n");

        // Restore fusion mode (save left chip in CONFIG_MODE)
        sensorMagnetometer_setOperationMode(currentMode);
    }
}

void sensorMagnetometer_setOperationMode(uint8_t mode)
{
    HAL_I2C_Mem_Write(&I2C_BNO055_Handle, BNO055_ADDR << 1, BNO055_OPR_MODE, I2C_MEMADD_SIZE_8BIT, &mode, 1, 1000);
    HAL_Delay(10); // small delay to allow mode switch to take effect
    currentMode = mode;
}

// 0xAA is the start byte

/**
  * Handler for the task.
  */
void sensorMagnetometer_handler(void *argument)
{
    for(;;)
    {
        // sensorMagnetometer_readACC_Vector();
        // sensorMagnetometer_readMAG_Vector();
        sensorMagnetometer_readGYRO_Vector();
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for demonstration purposes
    }
}

void sensorMagnetometer_readWhoAmI() {
    sensorMagnetometer_readChip(BNO055_WHO_AM_I, "Who Am I");
}

static void sensorMagnetometer_readChip(uint8_t regADDR, const char *name)
{   
    uint8_t receiveBuff = 0;
    uint8_t expected = 0;

    HAL_StatusTypeDef info;

    info = HAL_I2C_Mem_Read(&I2C_BNO055_Handle, BNO055_ADDR << 1, regADDR,
                            I2C_MEMADD_SIZE_8BIT, &receiveBuff, 1, 5000);

    if (info != HAL_OK) {
        printf("%s FAILED, HAL status: %d, I2C error: 0x%lX\r\n", name, info, HAL_I2C_GetError(&I2C_BNO055_Handle));
        HAL_I2C_DeInit(&I2C_BNO055_Handle);
        HAL_I2C_Init(&I2C_BNO055_Handle);
        return;
    }

    switch (regADDR) {
        case BNO055_WHO_AM_I: expected = 0xA0; break;
        case BNO055_ACC: expected = 0xFB; break;
        case BNO055_MAG: expected = 0x32; break;
        case BNO055_GYRO: expected = 0x0F; break;
        case BNO055_ST_RESULT: expected = 0x0F; break;
        default: expected = 0xff;
    }

    if(receiveBuff == expected)
    {
        printf("BNO055 %s OK: 0x%02X\r\n", name, receiveBuff);
    }
    else
    {
        printf("BNO055 %s FAILED: expected 0x%02X, got 0x%02X\r\n", name, expected, receiveBuff);
    }
}

void sensorMagnetometer_readMAG()
{   
    sensorMagnetometer_readChip(BNO055_MAG, "MAG");
}

void sensorMagnetometer_readACC()
{   
    sensorMagnetometer_readChip(BNO055_ACC, "ACC");
}

void sensorMagnetometer_readGYRO()
{   
    sensorMagnetometer_readChip(BNO055_GYRO, "GYRO");
}

void sensorMagnetometer_readSelfTest()
{   
    sensorMagnetometer_readChip(BNO055_ST_RESULT, "Self-Test Result");
}

static void BNO055_readVector(uint8_t startReg, const char *name, int16_t *xData, int16_t *yData, int16_t *zData)
{
    uint8_t data[6] = {0xF, 0xF, 0xF, 0xF, 0xF, 0xF}; // Initialize with invalid data for easier debugging
    HAL_StatusTypeDef info;

    if ((info = HAL_I2C_Mem_Read(
        &I2C_BNO055_Handle,
        BNO055_ADDR << 1,       // 7-bit addr shifted for HAL
        startReg,               // register to start reading from
        I2C_MEMADD_SIZE_8BIT,   // BNO055 uses 8-bit register addresses
        data,                   // output buffer
        6,                      // read 6 bytes (LSB+MSB for X, Y, Z)
        5000                    // timeout ms
    )) != HAL_OK) {
        printf("%s Transmit FAILED, HAL status: %d, I2C error: 0x%lX\r\n", name, info, HAL_I2C_GetError(&I2C_BNO055_Handle));
    }

    int16_t x = (int16_t)((data[1] << 8) | data[0]);
    int16_t y = (int16_t)((data[3] << 8) | data[2]);
    int16_t z = (int16_t)((data[5] << 8) | data[4]);
    *xData = x; *yData = y; *zData = z;

    printf("%s: X=%d, Y=%d, Z=%d\r\n", name, x, y, z);
}

void sensorMagnetometer_readACC_Vector()
{
    BNO055_readVector(BNO055_ACCX_LSB, "ACC", &IMU.acc_x, &IMU.acc_y, &IMU.acc_z);
}

void sensorMagnetometer_readMAG_Vector()
{
    BNO055_readVector(BNO055_MAGX_LSB, "MAG",  &IMU.mag_x, &IMU.mag_y, &IMU.mag_z);
}

void sensorMagnetometer_readGYRO_Vector()
{
    BNO055_readVector(BNO055_GYRX_LSB, "GYRO",  &IMU.gyro_x, &IMU.gyro_y, &IMU.gyro_z);
}

void sensorMagnetometer_readVectorDynamic(uint8_t startReg, uint8_t bytes, const char *name)
{
    HAL_StatusTypeDef info;
    uint8_t buffer[256] = {0};

    if ((info = HAL_I2C_Mem_Read(
        &I2C_BNO055_Handle,
        BNO055_ADDR << 1,       // 7-bit addr shifted for HAL
        startReg,               // register to start reading from
        I2C_MEMADD_SIZE_8BIT,   // BNO055 uses 8-bit register addresses
        buffer,                 // output buffer
        bytes,                   // number of bytes to read
        5000                    // timeout ms
    )) != HAL_OK) {
        printf("%s Transmit FAILED, HAL status: %d, I2C error: 0x%lX\r\n", name, info, HAL_I2C_GetError(&I2C_BNO055_Handle));
    }

    printf("%s: ", name);
    for (uint8_t i = 0; i < bytes; i++) {
        printf("%02X ", buffer[i]);
    }
    printf("\r\n");
}

uint8_t checkCalibration() {

    HAL_StatusTypeDef info;
    uint8_t calBuffer = 0;
    
    if ((info = HAL_I2C_Mem_Read(
        &I2C_BNO055_Handle,
        BNO055_ADDR << 1,       // 7-bit addr shifted for HAL
        BNO55_CALIB_STATE,               // register to start reading from
        I2C_MEMADD_SIZE_8BIT,   // BNO055 uses 8-bit register addresses
        &calBuffer,                 // output buffer
        1,                   // number of bytes to read
        5000                    // timeout ms
    )) != HAL_OK) {
        printf("Calibration Register Read FAILED, HAL status: %d, I2C error: 0x%lX\r\n", info, HAL_I2C_GetError(&I2C_BNO055_Handle));
    }

    uint8_t sys_cal  = (calBuffer >> 6) & 0x03;
    uint8_t gyro_cal = (calBuffer >> 4) & 0x03;
    uint8_t acc_cal  = (calBuffer >> 2) & 0x03;
    uint8_t mag_cal  = (calBuffer >> 0) & 0x03;

    printf("System: %d/3 | Accel: %d/3 | Mag: %d/3 | Gyro: %d/3\r\n",
           sys_cal, acc_cal, mag_cal, gyro_cal);

    // All three must be fully calibrated
    return (acc_cal == 3 && mag_cal == 3 && gyro_cal == 3);
}

// Save the 18 offset bytes out of the BNO055 into savedOffsets[]
static void BNO055_saveCalibrationOffsets()
{
    HAL_StatusTypeDef info;

    // Must be in CONFIGMODE to read offsets reliably
    sensorMagnetometer_setOperationMode(BNO055_OPR_MODE_CONFIG);
    HAL_Delay(20);

    if ((info = HAL_I2C_Mem_Read(
        &I2C_BNO055_Handle,
        BNO055_ADDR << 1,
        BNO055_CALIB,
        I2C_MEMADD_SIZE_8BIT,
        savedOffsets,
        CALIB_OFFSET_SIZE,
        5000
    )) != HAL_OK) {
        printf("Offset save FAILED, status: %d, err: 0x%lX\r\n",
               info, HAL_I2C_GetError(&I2C_BNO055_Handle));
        return;
    }

    printf("Calibration offsets saved\r\n");
}

// Write savedOffsets[] back into the BNO055 to skip motion calibration on next boot
static void BNO055_loadCalibrationOffsets()
{
    HAL_StatusTypeDef info;

    // Must be in CONFIGMODE to write offsets
    sensorMagnetometer_setOperationMode(BNO055_OPR_MODE_CONFIG);
    HAL_Delay(20);

    if ((info = HAL_I2C_Mem_Write(
        &I2C_BNO055_Handle,
        BNO055_ADDR << 1,
        BNO055_CALIB,
        I2C_MEMADD_SIZE_8BIT,
        savedOffsets,
        CALIB_OFFSET_SIZE,
        5000
    )) != HAL_OK) {
        printf("Offset load FAILED, status: %d, err: 0x%lX\r\n",
               info, HAL_I2C_GetError(&I2C_BNO055_Handle));
        return;
    }

    printf("Calibration offsets restored — skipping motion calibration\r\n");
}