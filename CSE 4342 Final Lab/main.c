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
#include "tm4c123gh6pm.h"
#include "i2c0.h"
#include "gpio.h"
#include "wait.h"
#include "clock.h"

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

//-----------------------------------------------------------------------------
// Function prototypes
//-----------------------------------------------------------------------------

void initHw();
void setNextLED();

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

    // Configure MCP23008 GPIO pins
    writeI2c0Register(addrMCP23008, 0x00, 0x00);        // Set all GPIOs as output
    writeI2c0Register(addrMCP23008, RegGPIO, 0x00);     // Initialize all GPIO outputs as 0

    while (true)
    {
       // Check if button is pressed
       if (getPinValue(PORTF, 4) == 0)
       {
           while (getPinValue(PORTF, 4) == 0); // Wait for button release
           isForward = !isForward; // Toggle sequence mode
       }
       setNextLED();
   }
}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

 // Initialize hardware function
void initHw()
{
    initI2c0();                         // Initialize I2c0 library
    initUart0();                        // Initialize Uart0 library
    initSystemClockTo40Mhz();           // Initialize system clock to 40 MHz

    // Configure PF4 for pushbutton using GPIO library
    enablePort(PORTF);                  // Enable Port F
    selectPinDigitalInput(PORTF, 4);    // Set PF4 as input
    enablePinPullup(PORTF, 4);          // Enable pull-up resistor for PF4


}

// LED control logic
void setNextLED()
{
    if(isForward)
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

int main(void)
{

	return 0;
}
