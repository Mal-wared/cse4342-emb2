/*
// Reaction Time Test w/ UART & Push Button
// Nicholas Nhat Tran
// 1002027150

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red LED:
//   PF1 drives an NPN transistor that powers the red LED
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// Pushbutton:
//   SW1 pulls pin PF4 low (internal pull-up is used)

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "clock.h"
#include "tm4c123gh6pm.h"
#include "wait.h"

// Bitband aliases
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define PUSH_BUTTON  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))

// PortF masks
#define RED_LED_MASK 2
#define GREEN_LED_MASK 8
#define PUSH_BUTTON_MASK 16

// Globals
uint8_t currentColor = 0b0001;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Blocking function that returns only when SW1 is pressed
void waitPbPress(void)
{
    while(PUSH_BUTTON);
}

// Initialize Hardware
void initHw(void)
{
    initSystemClockTo40Mhz();
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    _delay_cycles(3);

    GPIO_PORTF_DIR_R |= GREEN_LED_MASK | RED_LED_MASK;
    GPIO_PORTF_DIR_R &= ~PUSH_BUTTON_MASK;

    GPIO_PORTF_DEN_R |= PUSH_BUTTON_MASK | GREEN_LED_MASK | RED_LED_MASK;
    GPIO_PORTF_PUR_R |= PUSH_BUTTON_MASK;
}

void manageCurrentColor()
{
    if(currentColor > 0b0011)
    {
        currentColor = 0b0001;
    }
    else
    {
        currentColor = currentColor << 1;
    }
}



//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main()
{
    // Initialize hardware
    initHw();

    // Endless loop
    while(true)
    {
        changeColor();
    }
}
*/
