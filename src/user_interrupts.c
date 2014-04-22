// This is where the "user" interrupts handlers should go
// The *must* be declared in "user_interrupts.h"

#include "maindefs.h"
#ifndef __XC8
#include <timers.h>
#else
#include <plib/timers.h>
#endif
#include "user_interrupts.h"
#include "messages.h"
#include "debug.h"
#include "sensor_thread.h"
#include "arm_thread.h"


// A function called by the interrupt handler
// This one does the action I wanted for this program on a timer0 interrupt

void timer0_int_handler() {
    DEBUG_ON(TMR_DBG);
    DEBUG_OFF(TMR_DBG);
    // reset the timer
    WriteTimer0(0);
    // Send a sensor request
    ToMainHigh_sendmsg(0, MSGT_TIMER0, (void *) 0);
}

// A function called by the interrupt handler
// This one does the action I wanted for this program on a timer1 interrupt

void timer1_int_handler() {
//    DEBUG_ON(TMR_DBG);
//    DEBUG_OFF(TMR_DBG);
//    DEBUG_ON(TMR_DBG);
//    DEBUG_OFF(TMR_DBG);
//
//    // reset the timer
//    WriteTimer1(0x8000);
//    // Send an encoder request
//    ToMainHigh_sendmsg(0, MSGT_TIMER1, (void *) 0);
}