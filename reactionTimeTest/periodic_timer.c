// Periodic timer example
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <inttypes.h>
#include "clock.h"
#include "tm4c123gh6pm.h"
#include "wait.h"
#include "uart0.h"

// Pin bitbands
#define RED_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define PUSH_BUTTON  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))

// PortF masks
#define RED_LED_MASK 2
#define GREEN_LED_MASK 8
#define PUSH_BUTTON_MASK 16

#define TIMER_VALUE_STR_SIZE 12

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

bool oneshotDone = false;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks (TIMER0, TIMER1, GPIO PORT F)
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R0 | SYSCTL_RCGCTIMER_R1;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    _delay_cycles(3);

    // Configure LED pin & push button pin
    GPIO_PORTF_DIR_R |= RED_LED_MASK | GREEN_LED_MASK;
    GPIO_PORTF_DIR_R &= ~PUSH_BUTTON_MASK;
    GPIO_PORTF_DEN_R |= PUSH_BUTTON_MASK | RED_LED_MASK | GREEN_LED_MASK;
    GPIO_PORTF_PUR_R |= PUSH_BUTTON_MASK;

    /*
    // Configure Timer 1 as the time base
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_1_SHOT;          // configure for one-shot mode (count up)
    TIMER1_TAILR_R = 40000000;                       // set load value to 40e6 for 1 Hz interrupt rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts for timeout in timer module
    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    NVIC_EN0_R = 1 << (INT_TIMER1A-16);              // turn-on interrupt 37 (TIMER1A) in NVIC
    */
}

// Periodic timer
void timer1Isr(void)
{
    GREEN_LED = 1;
    oneshotDone = true;
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
}

bool buttonStillPressed = false;
int isValidPress()
{
    // Record current press
    // A valid press = if button is currently pressed AND it isn't still pressed from last time
    // Record if button is pressed for next instance (when button isn't pressed, return to not pressed)
    // Return if it's a valid press

    bool isPressedNow = !PUSH_BUTTON;
    bool validPress = isPressedNow && !buttonStillPressed;
    buttonStillPressed = isPressedNow;
    return validPress;
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    char timer_value_string[TIMER_VALUE_STR_SIZE];

    // Initialize hardware
    initHw();

    initUart0();
    setUart0BaudRate(115200, 40e6);

    // Endless loop
    while (true)
    {
        // start LED on red
        // start random one-shot timer
        // when timer ends, turn LED green and then set a real-time clock timer
        // when a button is pressed, stop RTC timer and display on UART
        // start on red again

        GREEN_LED = 0;
        RED_LED = 1;

        // TODO - MAKE TIMER RANDOM
        TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
        TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
        TIMER1_TAMR_R = TIMER_TAMR_TAMR_1_SHOT;          // configure for one-shot mode (count up)
        TIMER1_TAILR_R = 40000000;                       // set load value to 40e6 for 1 Hz interrupt rate
        TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts for timeout in timer module
        TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
        NVIC_EN0_R = 1 << (INT_TIMER1A-16);              // turn-on interrupt 37 (TIMER1A) in NVIC

        while(!oneshotDone);

        TIMER0_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
        TIMER0_CFG_R = TIMER_CFG_16_BIT;             // configure as 32-bit timer (A+B)
        TIMER0_TAMATCHR_R = TIMER_TAMATCHR_TAMR_M;       // configure as 32-bit timer (A+B)
        TIMER0_CTL_R |= TIMER_CTL_RTCEN;                 // set stall bit while debugging for RTC
        TIMER0_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
        TIMER0_TAV_R = 0;

        while(!isValidPress());

        /*snprintf(timer_value_string,            // The character buffer to write into
                 TIMER_VALUE_STR_SIZE,          // The maximum size of the buffer
                 "%" PRIu32,                    // The format specifier for unsigned 32-bit int
                 *((volatile uint32_t *)TIMER0_TAV_R));                 // The 32-bit integer value to convert
        */
        uint32_t time = (*((volatile uint32_t *)TIMER0_TAV_R))/40000000;
        putsUart0("hello");


    }
}
