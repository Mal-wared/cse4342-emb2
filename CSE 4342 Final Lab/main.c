// Lab 2: Interfacing MCP23008 with TM4C123G for LED Control
// Nicholas Nhat Tran
// 1002027150

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Serial Clock Line (SCL):
//   I2C0SCL (PB2) drives the SCL on the MCP23008
// Serial Data Line (SDA):
//   I2C0SDA (PB3) drives the SDA on the MCP23008
// Pull-Up Push Button
//   Pulls pin (PF4) low (internal pull-up is used)

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "tm4c123gh6pm.h"
#include "i2c0.h"
#include "gpio.h"
#include "wait.h"
#include "clock.h"
#include "uart0.h"
#include "icm20948.h"

#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define GREEN_LED_MASK 8

#define I2C0SCL PORTB,2    // I2C0 Clock (SCL) mapped to PB2
#define I2C0SDA PORTB,3    // I2C0 Data (SDA) mapped to PB3
#define addrMCP23008  0x20 // MCP23008 Address
#define addrMCP23008  0x20 // MCP23008 Address
#define RegGPIO  0x09      // GPIO register address

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------
bool isForward = true; // toggle between normal and reverse sequences
uint8_t currentLED = 0b00001;
float gyroOffsetX;
float gyroOffsetY;
float gyroOffsetZ;


//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize hardware function
void initHw()
{
    initSystemClockTo40Mhz();           // Initialize system clock to 40 MHz

    // Configure PF4 for pushbutton using GPIO library
    enablePort(PORTF);                  // Enable Port F
    selectPinDigitalInput(PORTF, 4);    // Set PF4 as input               // Enable Port F
    GPIO_PORTF_DIR_R |= GREEN_LED_MASK;
    GPIO_PORTF_DEN_R |= GREEN_LED_MASK;
    enablePinPullup(PORTF, 4);          // Enable pull-up resistor for PF4

}

// LED control logic
void setNextLED()
{
    if (isForward)
    {
        currentLED = (currentLED >> 1) | (currentLED << 4);
        writeI2c0Register(addrMCP23008, RegGPIO, currentLED);
    }
    else
    {
        currentLED = (currentLED << 1) | (currentLED >> 4);
        writeI2c0Register(addrMCP23008, RegGPIO, currentLED);
    }
    waitMicrosecond(500000);
}

void initICM20948()
{
    // Step 1: Switch to Register Bank 0
    writeI2c0Register(ICM20948_ADDR, REG_BANK_SEL, BANK0);
    waitMicrosecond(1000);

    // Step 2a: IMU reset
    writeI2c0Register(ICM20948_ADDR, PWR_MGMT_1, 0xC1);
    waitMicrosecond(100000);

    // Step 2b: Disable Sleep Mode & Select Best Clock
    writeI2c0Register(ICM20948_ADDR, PWR_MGMT_1, 0x01);
    waitMicrosecond(1000);

    // Step 3a: Switch to Register Bank 2
    writeI2c0Register(ICM20948_ADDR, REG_BANK_SEL, BANK2);
    waitMicrosecond(1000);

    // Step 3b: Output data rate start time alignment
    writeI2c0Register(ICM20948_ADDR, ODR_ALIGN_EN, 0x01);
    waitMicrosecond(1000);

    // Step 4a: Gyroscope configuration, sample rate divider = 0, page 59
    writeI2c0Register(ICM20948_ADDR, GYRO_SMPLRT_DIV, 0x00);
    waitMicrosecond(1000);

    // Step 4b: Enable Digital Low Pass Filter (DLPF) for Gyroscope
    writeI2c0Register(ICM20948_ADDR, GYRO_CONFIG_1, 0x01);
    waitMicrosecond(1000);
    // Bitfield explanation:
    //   Bits [5:3] = Gyro DLPF config (on)
    //   Bits [2:1] = Gyro full-scale = ±250 dps (scale factor = 131)
    //   Bit  [0]   = DLPF enabled

    // Step 5: Switch back to Register Bank 0
    writeI2c0Register(ICM20948_ADDR, REG_BANK_SEL, BANK0);
    waitMicrosecond(1000);

    // Step 6: Enable all gyroscope and accelerometer axes
    writeI2c0Register(ICM20948_ADDR, PWR_MGMT_2, 0x00);
    waitMicrosecond(1000);

    // Step 7: Switch back to Register Bank 2
    writeI2c0Register(ICM20948_ADDR, REG_BANK_SEL, BANK2);
    waitMicrosecond(1000);

    // Step 8a: Accelerometer configuration, sample rate divider = 0
    writeI2c0Register(ICM20948_ADDR, ACCEL_SMPLRT_DIV_1, 0x00);
    waitMicrosecond(1000);

    // Step 8b: Accelerometer configuration, sample rate divider = 0
    writeI2c0Register(ICM20948_ADDR, ACCEL_SMPLRT_DIV_2, 0x00);
    waitMicrosecond(1000);

    // Step 9: Accelerometer configuration, gyroscope range set and enable digital filter
    writeI2c0Register(ICM20948_ADDR, ACCEL_CONFIG, 0x01);
    waitMicrosecond(1000);
}

void readGyroY()
{
    char buffer[32];

    // Ensure we are on BANK0
    writeI2c0Register(ICM20948_ADDR, REG_BANK_SEL, BANK0);
    waitMicrosecond(1000);

    // Step 1: Read high byte of Y-axis gyroscope data
    uint8_t highByte = readI2c0Register(ICM20948_ADDR, GYRO_YOUT_H);

    snprintf(buffer, 32, "GYRO_YOUT_H: %hu\n", highByte);
    putsUart0(buffer);

    // Step 2: Read low byte of Y-axis gyroscope data
    uint8_t lowByte = readI2c0Register(ICM20948_ADDR, GYRO_YOUT_L);
    snprintf(buffer, 32, "GYRO_YOUT_L: %hu\n", lowByte);
    putsUart0(buffer);

    // Step 3: Combine high and low bytes into a signed 16-bit integer
    int16_t raw = (highByte << 8) | lowByte;

    // Bonus: Convert raw value to degrees per second
    float degPerSec = raw / 131.0;

    snprintf(buffer, 32, "GYRO_YOUT: %f\n", degPerSec);
    putsUart0(buffer);
}

void runControllerLoop()
{

}

void getGyroCalibrationValues()
{
    // Ensure we're on BANK0 for gyro readings
    writeI2c0Register(ICM20948_ADDR, REG_BANK_SEL, BANK0);
    waitMicrosecond(1000);

    char buffer[32];

    int32_t gyroCaliValX = 0;
    int32_t gyroCaliValY = 0;
    int32_t gyroCaliValZ = 0;

    uint8_t highByteX;
    uint8_t lowByteX;
    uint8_t highByteY;
    uint8_t lowByteY;
    uint8_t highByteZ;
    uint8_t lowByteZ;

    uint16_t i;
    for (i = 0; i < 2000; i++)
    {
        highByteX = readI2c0Register(ICM20948_ADDR, GYRO_XOUT_H);
        lowByteX = readI2c0Register(ICM20948_ADDR, GYRO_XOUT_L);
        int16_t gyroCaliValRawX = (highByteX << 8 | lowByteX);

        highByteY = readI2c0Register(ICM20948_ADDR, GYRO_YOUT_H);
        lowByteY = readI2c0Register(ICM20948_ADDR, GYRO_YOUT_L);
        int16_t gyroCaliValRawY = (highByteY << 8 | lowByteY);

        highByteZ = readI2c0Register(ICM20948_ADDR, GYRO_ZOUT_H);
        lowByteZ = readI2c0Register(ICM20948_ADDR, GYRO_ZOUT_L);
        int16_t gyroCaliValRawZ = (highByteZ << 8 | lowByteZ);


        gyroCaliValX += gyroCaliValRawX;
        gyroCaliValY += gyroCaliValRawY;
        gyroCaliValZ += gyroCaliValRawZ;

        waitMicrosecond(100);
    }
    gyroOffsetX = (float)gyroCaliValX / 2000.0f;
    snprintf(buffer, 32, "Gyro Offset (X): %f\n", gyroOffsetX);
    putsUart0(buffer);
    gyroOffsetY = (float)gyroCaliValY / 2000.0f;
    snprintf(buffer, 32, "Gyro Offset (Y): %f\n", gyroOffsetY);
    putsUart0(buffer);
    gyroOffsetZ = (float)gyroCaliValZ / 2000.0f;
    snprintf(buffer, 32, "Gyro Offset (Z): %f\n", gyroOffsetZ);
    putsUart0(buffer);
    waitMicrosecond(1000);
    GREEN_LED = 1;
}

void testRegisterWhoAmI()
{
    uint8_t whoami = readI2c0Register(ICM20948_ADDR, 0x00);
    char buffer[32];
    snprintf(buffer, 32, "WHO_AM_I: 0x%02X\n", whoami);
    putsUart0(buffer);
    if (whoami == 0xEA)
    {
        putsUart0("Connected to ICM-20948\n");
    }
    else
    {
        putsUart0("Failed to connect to ICM-20948\n");
    }
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    // interface PID with I2C
    // configure gyro
    // read values from gyro
    //      calculate tilt values

    // start with trying to get "WHO_AM_I"
    // if you can display on PuTTY, you're good-ish

    // write to bank
    // write to register
    // read from GYRO and EXCEL registers
    initHw();
    initUart0();
    initI2c0();
    initICM20948();

    GREEN_LED = 0;
    setUart0BaudRate(115200, 40e6);
    testRegisterWhoAmI();
    getGyroCalibrationValues();


    //uint8_t reg = 0x07f;
    //writeI2c0Register(reg, 0x00, 0x00);
    // Configure MCP23008 GPIO pins
    //writeI2c0Register(addrMCP23008, 0x00, 0x00);        // Set all GPIOs as output
    //writeI2c0Register(addrMCP23008, RegGPIO, 0x00);     // Initialize all GPIO outputs as 0

    while (true)
    {
        readGyroY();
        waitMicrosecond(500000);
    }
}

