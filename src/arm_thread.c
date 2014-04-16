#include "maindefs.h"
#include <stdio.h>
#include "arm_thread.h"
#include "messages.h"
#include "debug.h"
#include "my_uart.h"

// This is a "logical" thread that processes messages from the UART
// It is not a "real" thread because there is only the single main thread
// of execution on the PIC because we are not using an RTOS.

int arm_lthread(arm_thread_struct *uptr, int msgtype, int length, unsigned char *msgbuffer) {
    if (msgtype == MSGT_OVERRUN) {
        // TODO handle when buffer overruns.
    } else if (msgtype == MSGT_ARM_SEND) {
        // Send message via UART to ARM.
        uart_retrieve_buffer(length, msgbuffer);
    }
}