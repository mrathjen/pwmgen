//*****************************************************************************
//
// pwmgen.c - PWM signal generation example.
//
// Copyright (c) 2005-2012 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 9453 of the EK-LM3S8962 Firmware Package.
//
//*****************************************************************************

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "drivers/rit128x96x4.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/lm3s8962.h"
#include "driverlib/interrupt.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "utils/ustdlib.h"
#include <stdbool.h>

#include "driverlib/timer.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>PWM (pwmgen)</h1>
//!
//! This example application utilizes the PWM peripheral to output a 25% duty
//! cycle PWM signal and a 75% duty cycle PWM signal, both at 440 Hz.  Once
//! configured, the application enters an infinite loop, doing nothing while
//! the PWM peripheral continues to output its signals.
//
//*****************************************************************************

// PWM defines
#define PERIOD 2.27 // ms
#define UPPER_PERIOD 2.0 //ms
#define LOWER_PERIOD 1.0 //ms

#define PWM_INCREMENT 

// If you undefine DEBUG, remeber to rebuild the libraries too
#define DEBUG

// The rate of system ticks in Hz
#define TICK_RATE 1000
#define UART0_BAUD 115200
#define UART1_BAUD 115200

// Add this to the base address of GPIO data registers to unmask the whole byte
// see section 8.2.1.2 in the LM3S8962 datasheet.
#define ADDR_MASK 0x3FC

#define STRX(x) #x
#define STR(x) STRX(x) // convert integer macro x to string literal

//ScreenBuffer sbuff;

volatile unsigned long g_ulTickCount = 0;
volatile struct {
    bool up_press:1;
    bool down_press:1;
} Flags = {0, 0};

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif

//*****************************************************************************
// Debounce the select button: set a flag when it is pressed.
//*****************************************************************************
#define BUT_POLL_RATE 500 // Hz
#define BUT_DEAD_TIME 20  // ms
#define BUT_TICKS_PER_POLL ((TICK_RATE / BUT_POLL_RATE) == 0 ? 1 : (TICK_RATE / BUT_POLL_RATE) )
#define BUT_HIST_LENGTH ((BUT_DEAD_TIME * TICK_RATE) / (BUT_TICKS_PER_POLL * 1000 ))
#define BUT_HIST_MASK (0xffffffff << (BUT_HIST_LENGTH > 32 ? 32 : BUT_HIST_LENGTH  + 1) )
void DebounceSelect(void)
{
    // The most recent state of the switch is the LSB
    static unsigned long history0 = 0;
    static unsigned long history1 = 0;
    
    if ((g_ulTickCount % BUT_TICKS_PER_POLL) == 0)
    {
        history0 = (history0 << 1) | GPIOPinRead(GPIO_PORTE_BASE + GPIO_O_DATA, GPIO_PIN_0);
        history1 = (history1 << 1) | GPIOPinRead(GPIO_PORTE_BASE + GPIO_O_DATA, GPIO_PIN_1) >> 1;
        
        /*// In order to trigger a button pressed flag, the current reading*/
        /*// should be 0 and all the previous readings should be 1.*/
        if ((history0 | BUT_HIST_MASK) == 0xfffffffe )
        {
            Flags.up_press = 1;
        }
        if ((history1 | BUT_HIST_MASK) == 0xfffffffe )
        {
            Flags.down_press = 1;
        } 
    }
}

unsigned long upper_period, lower_period, inc_period, current_period, ulPeriod; 

//*****************************************************************************
// Depending on the button selected change the duty cycle of the PWM
//*****************************************************************************
void PWM_Change(void)
{
    if (Flags.up_press)
    {
        // Toggle the LED (PF0).
        HWREGBITB(GPIO_PORTF_BASE + ADDR_MASK, 0) ^= 1;
        
        // Increase the duty cycle
        if (current_period == lower_period) {
            current_period = upper_period;
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, current_period);
        }
        
    } else if (Flags.down_press) {
        // Toggle the LED (PF0).
        HWREGBITB(GPIO_PORTF_BASE + ADDR_MASK, 0) ^= 1;
        
        // Decrease the duty cycle
        if (current_period == upper_period) {
            current_period = lower_period;
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, current_period);
        }
    }
}



//*****************************************************************************
//
// Flags that contain the current value of the interrupt indicator as displayed
// on the OLED display.
//
//*****************************************************************************
unsigned long g_ulFlags;

//*****************************************************************************
//
// The interrupt handler for the first timer interrupt.
//
//*****************************************************************************
void
Timer0IntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Toggle the flag for the first timer.
    //
    HWREGBITW(&g_ulFlags, 0) ^= 1;

    // Toggle the LED (PF0).
    //HWREGBITB(GPIO_PORTF_BASE + ADDR_MASK, 0) ^= 1;
    
    //SysTickIntHandler();
}

//*****************************************************************************
// The System Tick interrupt handler is called every 1/TICK_RATE seconds.  The
// interrupt is cleared automatically by hardware.
//*****************************************************************************
void SysTickIntHandler(void)
{
    g_ulTickCount++;
    DebounceSelect();
    PWM_Change();

    // clear the button pressed flag
    Flags.up_press = 0;
    Flags.down_press = 0;
}

// Initialize the pwm
void 
pwm_init(void) {
    // Set up the clock to be used by the pwm
    SysCtlPWMClockSet(SYSCTL_PWMDIV_2);

    //
    // Enable the peripherals used by this example.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    
    //
    // Set GPIO F0 and G1 as PWM pins.  They are used to output the PWM0 and
    // PWM1 signals.
    //
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_0);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_1);
    
    //
    // Compute the PWM period based on the system clock.
    //
    // Unlocked State:  SysCtlClockGet() / 2000;
    // Locked State:    SysCtlClockGet() / 900;
    ulPeriod = SysCtlClockGet() / 100;
    upper_period = SysCtlClockGet() / 900;
    lower_period = SysCtlClockGet() / 2000;
    current_period = lower_period;
    
    //
    // Set the PWM period to 440 (A) Hz.
    //
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, ulPeriod);
    
    //
    // Set PWM0 to a duty cycle initializing lock to unlocked state
    //
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, lower_period);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, lower_period);
    
    //
    // Enable the PWM0 and PWM1 output signals.
    //
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);
    
    //
    // Enable the PWM generator.
    //
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
}

void
up_down_but_init(void) {
    // Enable the GPIO port that is used for up and down button.
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOF;
    
    // Setup GPIO pin for up and down buttons
    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPU);
    
    // Enable the peripherals used 
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    
    // Configure SysTick to periodically interrupt.
    SysTickPeriodSet(SysCtlClockGet() / TICK_RATE);
    SysTickIntEnable();
    SysTickEnable();
}

//*****************************************************************************
//
// This example demonstrates how to setup the PWM block to generate signals.
//
//*****************************************************************************

int
main(void)
{
  
  // PW0: 560 micro secs
  // PW1: 1.7 ms
    
    //
    // Set the clocking to run directly from the crystal.
    //
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_8MHZ);
    
    // Enable the GPIO pin for the LED (PF0).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    //GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0);
    //GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA,
    //                 GPIO_PIN_TYPE_STD_WPU);
    
    //
    // Initialize the OLED display.
    //
    RIT128x96x4Init(1000000);

    //
    // Clear the screen and tell the user what is happening.
    //
    RIT128x96x4StringDraw("Generating PWM", 18, 24, 15);
    RIT128x96x4StringDraw("on PF0 and PG1", 18, 32, 15);

    // Enable processor interrupts.
    IntMasterEnable();
    
    pwm_init();
    up_down_but_init();
    
    //
    // Enable the peripherals used by this example.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    
    //
    // Configure the two 32-bit periodic timers.
    //
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet());
    
    //
    // Setup the interrupts for the timer timeouts.
    //
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    
     //
    // Enable the timers.
    //
    TimerEnable(TIMER0_BASE, TIMER_A);

    //
    // Loop forever while the PWM signals are generated.
    //
    while(1)
    {
    }
}

