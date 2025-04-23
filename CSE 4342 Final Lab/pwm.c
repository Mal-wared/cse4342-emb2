// PWM Motor Library
// Nicholas Nhat Tran
// 1002027150

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL with LCD Interface
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// PWM-Controlled Motor:
//   M1PWM5 (PF1) drives an NPN transistor that powers the motor

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <pwm.h>
#include <stdint.h>
#include "tm4c123gh6pm.h"

// PortF masks
#define MOTOR_MASK 2

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize RGB
void initPwm()
{
    // Enable clocks
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    _delay_cycles(3);

    // Configure three LEDs
    GPIO_PORTF_DEN_R |= MOTOR_MASK;
    GPIO_PORTF_AFSEL_R |= MOTOR_MASK;
    GPIO_PORTF_PCTL_R &= ~(GPIO_PCTL_PF1_M);
    GPIO_PORTF_PCTL_R |= GPIO_PCTL_PF1_M1PWM5;

    // Configure PWM module 1 to drive motor
    // MOTOR   on M1PWM5 (PF1), M1PWM2b
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;                // reset PWM1 module
    SYSCTL_SRPWM_R = 0;                              // leave reset state
    PWM1_2_CTL_R = 0;                                // turn-off PWM1 generator 2 (drives outs 4 and 5)
    PWM1_2_GENA_R = PWM_1_GENA_ACTCMPBD_ONE | PWM_1_GENA_ACTLOAD_ZERO;
    PWM1_2_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO;
                                                     // output 5 on PWM1, gen 2b, cmpb
    PWM1_2_LOAD_R = 20000;                            // set frequency to 40 MHz sys clock / 2 / 20000 = 1 kHz

    PWM1_2_CMPA_R = 0;
    PWM1_2_CMPB_R = 0;                               // red off (0=always low, 1023=always high)

    PWM1_2_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM1 generator 2
    PWM1_ENABLE_R = PWM_ENABLE_PWM5EN | PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;
                                                     // enable outputs
}

void setThreshold(uint16_t thres)
{
    PWM1_2_CMPA_R = -thres;
    PWM1_2_CMPB_R = thres;

}


