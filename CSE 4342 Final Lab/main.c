// Final Project: Self-Balancing Robot using ICM-20948 IMU sensor
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
#include <math.h>
#include "tm4c123gh6pm.h"
#include "i2c0.h"
#include "gpio.h"
#include "wait.h"
#include "clock.h"
#include "uart0.h"
#include "icm20948.h"
#include "pwm.h"

#define IN1      (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 4*4))) // Port B4
#define IN2      (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 5*4))) // Port B5
#define IN3      (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 4*4))) // Port C4
#define IN4      (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 5*4))) // Port C5
#define IN1_MASK 16
#define IN2_MASK 32
#define IN3_MASK 16
#define IN4_MASK 32

#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define GREEN_LED_MASK 8

#define I2C0SCL PORTB,2    // I2C0 Clock (SCL) mapped to PB2
#define I2C0SDA PORTB,3    // I2C0 Data (SDA) mapped to PB3
#define addrMCP23008  0x20 // MCP23008 Address
#define addrMCP23008  0x20 // MCP23008 Address
#define RegGPIO  0x09      // GPIO register address

#define PID_KP  5000.0f
#define PID_SP  -2.0f

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------
bool isForward = true; // toggle between normal and reverse sequences
uint8_t currentLED = 0b00001;
float gyroOffsetX, gyroOffsetY, gyroOffsetZ;
float gyroX, gyroY, gyroZ;
float acceX, acceY, acceZ;
float pitch = 0.0f;
float roll = 0.0f;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize hardware function
void initHw()
{
    initSystemClockTo40Mhz();           // Initialize system clock to 40 MHz

    // Configure PF4 for pushbutton using GPIO library
    enablePort(PORTF);                  // Enable Port F
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1 | SYSCTL_RCGCGPIO_R2;
    _delay_cycles(3);
    selectPinDigitalInput(PORTF, 4); // Set PF4 as input               // Enable Port F
    GPIO_PORTF_DIR_R |= GREEN_LED_MASK;
    GPIO_PORTF_DEN_R |= GREEN_LED_MASK;

    GPIO_PORTB_DIR_R |= IN1_MASK | IN2_MASK;
    GPIO_PORTB_DEN_R |= IN1_MASK | IN2_MASK;
    GPIO_PORTC_DIR_R |= IN3_MASK | IN4_MASK;
    GPIO_PORTC_DEN_R |= IN3_MASK | IN4_MASK;
    enablePinPullup(PORTF, 4);          // Enable pull-up resistor for PF4

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

void initTimer()
{
    // Enable Timer1 Clock
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    // Dummy read/wait to ensure clock is active
    while ((SYSCTL_PRTIMER_R & SYSCTL_PRTIMER_R1) == 0)
    {
    };

    // Disable Timer A during configuration
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;

    // Configure for 32-bit timer mode
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;

    // Configure Timer A for Periodic mode, default down-count
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;

    // Set the load value to maximum for longest period before wrap-around
    TIMER1_TAILR_R = 0xFFFFFFFF;

    // Clear any interrupts (we are polling the value)
    TIMER1_IMR_R = 0;
    TIMER1_ICR_R = TIMER_ICR_TATOCINT; // Clear timeout flag just in case

    // Enable Timer A
    TIMER1_CTL_R |= TIMER_CTL_TAEN;
}

void getGyroAndAccelerometerValues()
{
    uint8_t highByteX = readI2c0Register(ICM20948_ADDR, GYRO_XOUT_H);
    uint8_t lowByteX = readI2c0Register(ICM20948_ADDR, GYRO_XOUT_L);
    uint8_t highByteY = readI2c0Register(ICM20948_ADDR, GYRO_YOUT_H);
    uint8_t lowByteY = readI2c0Register(ICM20948_ADDR, GYRO_YOUT_L);
    uint8_t highByteZ = readI2c0Register(ICM20948_ADDR, GYRO_ZOUT_H);
    uint8_t lowByteZ = readI2c0Register(ICM20948_ADDR, GYRO_ZOUT_L);

    int16_t gyroRawX = (highByteX << 8 | lowByteX);
    int16_t gyroRawY = (highByteY << 8 | lowByteY);
    int16_t gyroRawZ = (highByteZ << 8 | lowByteZ);

    gyroX = ((float) gyroRawX - gyroOffsetX) / 131.0f;
    gyroY = ((float) gyroRawY - gyroOffsetY) / 131.0f;
    gyroZ = ((float) gyroRawZ - gyroOffsetZ) / 131.0f;

    /******** DEBUGGING ********
     char buffer[32];
     snprintf(buffer, 32, "GYRO_XOUT: %f\n", gyroX);
     putsUart0(buffer);
     snprintf(buffer, 32, "GYRO_YOUT: %f\n", gyroY);
     putsUart0(buffer);
     snprintf(buffer, 32, "GYRO_ZOUT: %f\n", gyroZ);
     putsUart0(buffer);
     ********* DEBUGGING ********/

    highByteX = readI2c0Register(ICM20948_ADDR, ACCEL_XOUT_H);
    lowByteX = readI2c0Register(ICM20948_ADDR, ACCEL_XOUT_L);
    highByteY = readI2c0Register(ICM20948_ADDR, ACCEL_YOUT_H);
    lowByteY = readI2c0Register(ICM20948_ADDR, ACCEL_YOUT_L);
    highByteZ = readI2c0Register(ICM20948_ADDR, ACCEL_ZOUT_H);
    lowByteZ = readI2c0Register(ICM20948_ADDR, ACCEL_ZOUT_L);

    int16_t acceRawX = (highByteX << 8 | lowByteX);
    int16_t acceRawY = (highByteY << 8 | lowByteY);
    int16_t acceRawZ = (highByteZ << 8 | lowByteZ);

    acceX = (float) acceRawX / 16384;
    acceY = (float) acceRawY / 16384;
    acceZ = (float) acceRawZ / 16384;

    /******** DEBUGGING ********
     snprintf(buffer, 32, "ACCE_XOUT: %f\n", acceX);
     putsUart0(buffer);
     snprintf(buffer, 32, "ACCE_YOUT: %f\n", acceY);
     putsUart0(buffer);
     snprintf(buffer, 32, "ACCE_ZOUT: %f\n", acceZ);
     putsUart0(buffer);
     ********* DEBUGGING ********/
}

void calculatePitchAndRoll()
{
    static uint32_t prevTimerVal = 0; // Stores value from previous call
    static bool firstRun = true;             // Flag for first execution
    float dt = 0.0f;                          // Initialize dt

    uint32_t currentTimerVal = TIMER1_TAR_R;

    if (!firstRun)
    {
        // Calculate elapsed ticks (handling wrap-around for DOWN counter)
        uint32_t elapsedTicks;
        if (currentTimerVal <= prevTimerVal)
        {
            // Normal down-count case (no wrap-around)
            elapsedTicks = prevTimerVal - currentTimerVal;
        }
        else
        {
            // Wrap-around occurred (Timer counted down past 0 and reloaded)
            elapsedTicks = prevTimerVal + (0xFFFFFFFF - currentTimerVal + 1);
        }

        // Store current value for the NEXT call
        prevTimerVal = currentTimerVal;

        // Convert elapsed ticks to seconds
        dt = (float) elapsedTicks / 40000000;

        // Invalid dt cases
        if (dt > 0.1f) // If dt too large, cap it
        {
            dt = 0.1f;
        }
        else if (dt <= 0.0f) // If dt is somehow zero or negative, use a small positive value
        {
            dt = 0.001f; // Prevents division by zero/filter instability
        }
    }
    else
    {
        // dt calculation requires a previous_timer_value,
        prevTimerVal = currentTimerVal;
        firstRun = false;
        dt = 0.013762f; // Use a nominal dt (e.g., 10ms) for the very first calculation
    }

    getGyroAndAccelerometerValues();

    float measuredPitchAngle = (atan(acceX / sqrt(acceY * acceY + acceZ * acceZ)) * 1 / (M_PI / 180));
    //float measuredRollAngle = (atan(acceY / sqrt(acceX * acceX + acceZ * acceZ)) * 1 / (M_PI / 180));

    float lowpassPitch = pitch * 0.7 + measuredPitchAngle * 0.3;
    //float lowpassRoll = roll * 0.7 + measuredRollAngle * 0.3;

    // Complementary filter

    pitch = (0.96f) * (pitch + gyroY * dt) + (lowpassPitch * 0.04f);
    //roll = (0.97f) * (roll + gyroX * dt) + (lowpassRoll * 0.03f);

    char buffer[32];
    /*
    snprintf(buffer, 32, "dt: %f\n", dt);
    putsUart0(buffer);
    snprintf(buffer, 32, "roll: %f\n", roll);
    putsUart0(buffer);
    */
    snprintf(buffer, 32, "pitch: %f\n", pitch);
    putsUart0(buffer);

    float error = PID_SP - pitch;
    float output = PID_KP * error;

    if(output < 0.0f)
    {
        IN1 = 1; IN2 = 0; IN3 = 0; IN4 = 1;
    }
    else
    {

        IN1 = 0; IN2 = 1; IN3 = 1; IN4 = 0;

    }

    float scaledOutput = fabsf(output);
    float scaledOutputRounded = roundf(scaledOutput);
    if(scaledOutputRounded > 19999)
    {
        scaledOutputRounded = 19999;
    }
    setThreshold(scaledOutputRounded);

}

// Getting calibration values to reduce gyroscope drifting
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
    gyroOffsetX = (float) gyroCaliValX / 2000.0f;
    snprintf(buffer, 32, "Gyro Offset (X): %f\n", gyroOffsetX);
    putsUart0(buffer);
    gyroOffsetY = (float) gyroCaliValY / 2000.0f;
    snprintf(buffer, 32, "Gyro Offset (Y): %f\n", gyroOffsetY);
    putsUart0(buffer);
    gyroOffsetZ = (float) gyroCaliValZ / 2000.0f;
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
    initTimer();
    initPwm();

    GREEN_LED = 0;
    setUart0BaudRate(115200, 40e6);
    testRegisterWhoAmI();
    getGyroCalibrationValues();
    // Set threshold (Duty Cycle = Threshold / Load, Load = 20000)

    // Soft-start, increase duty cycle from 50% to 95%
    // NOTE: Starting the motor at 40% duty cycle will not work for most ERB lab motors, so 50% or higher must be utilized

    /*
    IN1 = 1; IN2 = 0; IN3 = 0; IN4 = 1;
    setThreshold(16000);
    while(true);
    */


    // Run calculations once to configure dt correctly
    calculatePitchAndRoll();
    waitMicrosecond(10000); // Small delay before loop starts


    while (true)
    {
        calculatePitchAndRoll();
        waitMicrosecond(10000);
    }


}

