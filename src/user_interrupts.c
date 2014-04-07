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

// A function called by the interrupt handler
// This one does the action I wanted for this program on a timer0 interrupt

void timer0_int_handler() {
    // reset the timer
    WriteTimer0(0);

    DEBUG_ON(TMR0_DBG);
    DEBUG_OFF(TMR0_DBG);

    // Send another command to the motorcontroller
    if (switchStatesCounter == switch_states) {
        DEBUG_ON(TMR0_DBG);
        DEBUG_OFF(TMR0_DBG);
        unsigned data[6];
        unsigned char length = 6;
        ToMainHigh_sendmsg(length, MSGT_TIMER0, (void *) data);
    }
}

// A function called by the interrupt handler
// This one does the action I wanted for this program on a timer1 interrupt

void timer1_int_handler() {
    unsigned int result;

    // read the timer and then send an empty message to main()
#ifdef __USE18F2680
    LATBbits.LATB1 = !LATBbits.LATB1;
#endif

    result = ReadTimer1();
    ToMainLow_sendmsg(0, MSGT_TIMER1, (void *) 0);

    // reset the timer
    WriteTimer1(0);
}