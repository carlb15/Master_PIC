#include "maindefs.h"
#ifndef __XC8
#include <usart.h>
#else
#include <plib/usart.h>
#endif
#include "my_uart.h"
#include "debug.h"

static uart_comm *uc_ptr;

void uart_recv_int_handler() {

    // Write a byte (one character) to the usart transmit buffer.
    // If 9-bit mode is enabled, the 9th bit is written from the field TX_NINE,
    // found in a union of type USART

#ifdef __USE18F26J50
    if (DataRdy1USART()) {
        uc_ptr->buffer[uc_ptr->buflen] = Read1USART();
#else
#ifdef __USE18F46J50
    if (DataRdy1USART()) {
        uc_ptr->buffer[uc_ptr->buflen] = Read1USART();
#else
    if (DataRdyUSART()) {
        // uc_ptr->buffer[uc_ptr->buflen] = ReadUSART();
#endif
#endif

        DEBUG_ON(USART_ISR);
        DEBUG_OFF(USART_ISR);
        DEBUG_ON(USART_ISR);
        DEBUG_OFF(USART_ISR);

        uartData = RCREG;
        uart_send_int_handler(uartData);

        //        uc_ptr->buflen++;
        //        // check if a message should be sent
        //        if (uc_ptr->buflen == MAXUARTBUF) {
        //            ToMainLow_sendmsg(uc_ptr->buflen, MSGT_UART_DATA, (void *) uc_ptr->buffer);
        //            uc_ptr->buflen = 0;
        //        }
    }
    //
    //
    //
    //#ifdef __USE18F26J50
    //    if (USART1_Status.OVERRUN_ERROR == 1) {
    //#else
    //#ifdef __USE18F46J50
    //    if (USART1_Status.OVERRUN_ERROR == 1) {
    //#else
    //    if (USART_Status.OVERRUN_ERROR == 1) {
    //#endif
    //#endif
    //        // we've overrun the USART and must reset
    //        // send an error message for this
    //        RCSTAbits.CREN = 0;
    //        RCSTAbits.CREN = 1;
    //        ToMainLow_sendmsg(0, MSGT_OVERRUN, (void *) 0);
    //    }
}

void uart_send_int_handler(unsigned char data) {

    WriteUSART(data);

}

void init_uart_recv(uart_comm *uc) {
    uc_ptr = uc;
    uc_ptr->buflen = 0;
}