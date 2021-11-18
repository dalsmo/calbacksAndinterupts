#include "timer.h"
#include <xc.h>             /* contains Vector Name/Number Macros */
#include <sys/attribs.h>

int register_Callback(functionToCall_t TC_temp, int delay_in_10ms){
    TC = TC_temp;
    // Turn off the timer, otherwise we will get unsuported state in PIC32
    T2CONbits.TON = 0; 
    // Set T2 period in decades of 10ms, this will do for now
    PR2 = 390*delay_in_10ms;
    // Clear counter
    TMR2 = 0;
    // Enable interrupts from Timer 2
    IEC0bits.T2IE = 1;
    //enable the timer
    T2CONbits.TON = 1;
    
}

int execute_Callback(void){
    TC();
} 

int initialise_Timer(void){
    /* Initialize Timer 2 Peripheral Settings */
    // Turn off the timer
    T2CONbits.TON = 0;
    // Pre-Scale = 1:256 (T2Clk: 39062.5Hz)
    T2CONbits.TCKPS = 7;
    // Set T2 period ~ 100mS
    PR2 = 3906;
    // Clear counter
    TMR2 = 0;

    /* Initialize Timer 2 Interrupt Controller Settings */
    // Set the interrupt priority to 4
    IPC2bits.T2IP = 7;
    // Reset the Timer 2 interrupt flag
    IFS0bits.T2IF = 0;
    // Enable interrupts from Timer 2
    //IEC0bits.T2IE = 1; // only do when we have a callback to call
  
    /* Set Interrupt Controller for multi-vector mode */
    INTCONSET = _INTCON_MVEC_MASK;

    /* Enable Interrupt Exceptions */
    // set the CP0 status IE bit high to turn on interrupts globally
    __builtin_enable_interrupts();

    /* Enable the peripheral */
    T2CONbits.TON = 1;    
}