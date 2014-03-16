#include "maindefs.h"
#ifndef __XC8
#include <usart.h>
#else
#include <plib/usart.h>
#endif
#include "my_uart.h"
#include "debug.h"

static uart_comm *uc_ptr;
static unsigned char buf[10];
static unsigned char count = 0;

void uart_recv_int_handler() {

    // Write a byte (one character) to the usart transmit buffer.
    // If 9-bit mode is enabled, the 9th bit is written from the field TX_NINE,
    // found in a union of type USART

#ifdef __USE18F26J50
    if (DataRdy1USART()) {
        uc_ptr->Rx_buffer[uc_ptr->Rx_buflen] = Read1USART();
#else
#ifdef __USE18F46J50
    if (DataRdy1USART()) {
        uc_ptr->Rx_buffer[uc_ptr->Rx_buflen] = Read1USART();
#else
    if (DataRdyUSART()) {

        char data = ReadUSART();

#endif
#endif

        if (!msgtype_flag) {
            msgtype = SENSOR_MSGTYPE;
        }

        switch (msgtype) {

            case SENSOR_MSGTYPE:
                if (!msgtype_flag) {
                    uc_ptr->Rx_buffer[0] = data;
                    uc_ptr->Rx_buflen++;
                    msgtype_flag = 1;
                    sendToSensorPIC_flag = 1;
                    msgtype = SENSOR_LENGTH;
                }
                break;

            case SENSOR_LENGTH:
                if (msgtype_flag && sendToSensorPIC_flag) {
                    uc_ptr->Rx_buffer[uc_ptr->Rx_buflen] = data;
                    uc_ptr->Rx_buflen++;
                    msg_flag = 1;
                    msgtype = CHECKSUM;
                } else {
                    msgtype = SENSOR_MSGTYPE;
                    msgtype_flag = 0;
                }
                break;

            case CHECKSUM:

                if (msg_flag) {

                    uc_ptr->Rx_buffer[uc_ptr->Rx_buflen] = data;
                    unsigned char checkSum = 0;
                    unsigned char bufLength = uc_ptr->Rx_buffer[1];

                    // Check for correct checksum.
                    if (bufLength == 0 && sendToSensorPIC_flag && uc_ptr->Rx_buffer[2] == SENSOR_MSGTYPE) {
                        checkSum = 0xaa;
                    } else {
                        // Check if the entire message was passsed correctly.
                        int i = 2;

                        for (; i < uc_ptr->Rx_buffer[1] + 2; i++) {
                            checkSum ^= uc_ptr->Rx_buffer[i];
                        }
                    }


                    if (checkSum != uc_ptr->Rx_buffer[uc_ptr->Rx_buflen]) {
                        // Drop the message
                        uc_ptr->Rx_buflen = 0;
                        // TODO Generate Error Message for Incorrect Message
                        //                 ToMainHigh_sendmsg(uc_ptr->Rx_buflen, MSGT_I2C_DATA, (void *) uc_ptr->Rx_buffer);
                    } else if (sendToSensorPIC_flag) {
                        uc_ptr->Rx_buflen++;
                        // Collect Motor Encoder Data
                        ToMainHigh_sendmsg(uc_ptr->Rx_buflen, MSGT_I2C_SEND, (void *) uc_ptr->Rx_buffer);
                    }

                    msgtype_flag = 0;
                    sendToSensorPIC_flag = 0;
                    uc_ptr->Rx_buflen = 0;
                    msg_flag = 0;
                    msgtype = SENSOR_MSGTYPE;
                } else {
                    msgtype = SENSOR_MSGTYPE;
                    msgtype_flag = 0;
                    sendToSensorPIC_flag = 0;
                    uc_ptr->Rx_buflen = 0;
                    msg_flag = 0;
                }
                break;

            case MESSAGE:
                if (uc_ptr->Rx_buflen == uc_ptr->Rx_buffer[1]) {
                    msg_flag = 1;
                    msgtype = CHECKSUM;
                } else {
                    msgtype = MESSAGE;
                }
                break;

            case MOTOR_COMMAND_MSGTYPE:
                break;

            case MOTOR_COMMAND_LENGTH:
                break;



            default:
                break;

        }


        // Basic implementation for 0xAA

        //        if (uc_ptr->Rx_buflen == 2) {
        //
        //            uc_ptr->Rx_buffer[uc_ptr->Rx_buflen] = data;
        //            uc_ptr->Rx_buflen++;
        //
        //            if (uc_ptr->Rx_buffer[0] == 0xaa && uc_ptr->Rx_buffer[1] == 0 && uc_ptr->Rx_buffer[2] == 0xaa) {
        //                ToMainHigh_sendmsg(uc_ptr->Rx_buflen, MSGT_I2C_SEND, (void *) uc_ptr->Rx_buffer);
        //                DEBUG_ON(CHECKSUM_DEBUG);
        //                DEBUG_OFF(CHECKSUM_DEBUG);
        //            }
        //
        //            uc_ptr->Rx_buflen = 0;
        //
        //        } else {
        //            uc_ptr->Rx_buffer[uc_ptr->Rx_buflen] = data;
        //            uc_ptr->Rx_buflen++;
        //        }








        //        if (!msgtype_flag && data == 0xaa) {
        //            uc_ptr->Rx_buffer[0] = data;
        //            msgtype_flag = 0x1;
        //            sendToSensorPIC_flag = 0x1;
        //            uc_ptr->Rx_buflen++;
        //        }
        ////        else if (!msgtype_flag && data == 0x01) {
        ////            uc_ptr->Rx_buffer[0] = data;
        ////            msgtype_flag = 0x1;
        ////            sendToMotorPIC_flag = 0x1;
        ////            uc_ptr->Rx_buflen++;
        ////        }
        //
        //        else if (sendToSensorPIC_flag && data == 0x0) {
        //            uc_ptr->Rx_buffer[uc_ptr->Rx_buflen] = data;
        //            // Iterate thru the length and data bytes.
        //            uc_ptr->Rx_buflen++;
        //        }
        ////        else if (sendToMotorPIC_flag && uc_ptr->Rx_buflen < 0x08) {
        ////            uc_ptr->Rx_buffer[uc_ptr->Rx_buflen] = data;
        ////            // Iterate thru the length and data bytes.
        ////            uc_ptr->Rx_buflen++;
        ////        }
        //
        //        else if (!checksum_flag && uc_ptr->Rx_buflen == uc_ptr->Rx_buffer[1] - 1) {
        //            uc_ptr->Rx_buffer[uc_ptr->Rx_buflen] = data;
        //
        //            // Check if the entire message was passsed correctly.
        //            checksum_flag = 0x1;
        //            int i = 0;
        //            unsigned char checkSum = 0x0;
        //
        //            for (; i < uc_ptr->Rx_buffer[1] - 1; i++) {
        //                checkSum ^= uc_ptr->Rx_buffer[i];
        //            }
        //
        //            if (checkSum != uc_ptr->Rx_buffer[uc_ptr->Rx_buflen]) {
        //                // Drop the message
        //                uc_ptr->Rx_buflen = 0;
        //
        //                // TODO Generate Error Message for Incorrect Message
        //                //                 ToMainHigh_sendmsg(uc_ptr->Rx_buflen, MSGT_I2C_DATA, (void *) uc_ptr->Rx_buffer);
        //            }
        //
        ////            else if (sendToMotorPIC_flag) {
        ////                uc_ptr->Rx_buflen++;
        ////                // Send command to motorcontroller
        ////                ToMainHigh_sendmsg(uc_ptr->Rx_buflen, MSGT_I2C_SEND, (void *) uc_ptr->Rx_buffer);
        ////            }
        //
        //            else if (sendToSensorPIC_flag) {
        //                uc_ptr->Rx_buflen++;
        //                // Collect motor data
        //                ToMainHigh_sendmsg(uc_ptr->Rx_buflen, MSGT_I2C_SEND, (void *) uc_ptr->Rx_buffer);
        //            }
        //            //reset flags for next message.
        //            msgtype_flag = 0x0;
        //            checksum_flag = 0x0;
        //            sendToMotorPIC_flag = 0x0;
        //            sendToSensorPIC_flag = 0x0;
        //            uc_ptr->Rx_buflen = 0;
        //        }
    }

#ifdef __USE18F26J50
    if (USART1_Status.OVERRUN_ERROR == 1) {
#else
#ifdef __USE18F46J50
    if (USART1_Status.OVERRUN_ERROR == 1) {
#else
    if (USART_Status.OVERRUN_ERROR == 1) {
#endif
#endif
        // we've overrun the USART and must reset
        // send an error message for this

        RCSTAbits.CREN = 0;
        RCSTAbits.CREN = 1;
        ToMainLow_sendmsg(0, MSGT_OVERRUN, (void *) 0);
    }
}

void uart_send_int_handler() {

    if (uc_ptr->Tx_buflen == uc_ptr->msg_length) {
        PIE1bits.TX1IE = 0; // Clear TXIE to end write.
        uc_ptr->Tx_buflen = 0;
    } else {
        WriteUSART(uc_ptr->Tx_buffer[uc_ptr->Tx_buflen]);
        uc_ptr->Tx_buflen++;
    }
}

void init_uart_recv(uart_comm *uc) {
    uc_ptr = uc;
    uc_ptr->Tx_buflen = 0;
    uc_ptr->Rx_buflen = 0;
    uc_ptr->msg_length = 0;
}

void uart_retrieve_buffer(int length, unsigned char* msgbuffer) {

    uc_ptr->Tx_buflen = 0;
    uc_ptr->msg_length = length;

    int i = 0;
    for (; i < length + 1; i++) {
        uc_ptr->Tx_buffer[i] = msgbuffer[i];
    }
}