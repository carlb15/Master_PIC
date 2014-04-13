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
        // Checks the valid message type within the state machine.
        checkForValidMsgType(data);
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

/**
 * Checks if part of the message has been sent correctly.
 * @param data
 *      Part of the UART message.
 */
void checkForValidMsgType(unsigned char data) {

    // Send to intial state
    if (!msgtype_flag && data == SENSOR_REQUEST) {
        // Intial sensor request
        msgtype = SENSOR_REQUEST;
    } else if (!msgtype_flag && data == MOTOR_COMMAND) {
        // Initial motor command.
        msgtype = MOTOR_COMMAND;
    } else if (!msgtype_flag && data == COMMAND_ACK) {
        // Initial command ACK state.
        msgtype = COMMAND_ACK;
    } else if (!msgtype_flag && data == COMMAND_NACK) {
        // Initial command NACK state.
        msgtype = COMMAND_NACK;
    } else if (!msgtype_flag && data == ENCODER_REQUEST) {
        // Initial encoder request
        msgtype = ENCODER_REQUEST;
    }

    switch (msgtype) {

        case COMMAND_ACK:
            if (!msgtype_flag && data == COMMAND_ACK) {
                uc_ptr->Rx_buffer[0] = data;
                uc_ptr->Rx_buflen++;
                msgtype_flag = 1;
                msgtype = ACK_LENGTH;
            }
            break;

        case COMMAND_NACK:
            if (!msgtype_flag && data == COMMAND_NACK) {
                uc_ptr->Rx_buffer[0] = data;
                uc_ptr->Rx_buflen++;
                msgtype_flag = 1;
                msgtype = NACK_LENGTH;
            }

        case SENSOR_REQUEST:
            if (!msgtype_flag && data == SENSOR_REQUEST) {
                uc_ptr->Rx_buffer[0] = data;
                uc_ptr->Rx_buflen++;
                msgtype_flag = 1;
                sendToSensorPIC_flag = 1;
                msgtype = SENSOR_LENGTH;
            }
            break;

        case MOTOR_COMMAND:
            if (!msgtype_flag && data == MOTOR_COMMAND) {
                uc_ptr->Rx_buffer[0] = data;
                uc_ptr->Rx_buflen++;
                msgtype_flag = 1;
                sendToMotorPIC_flag = 1;
                msgtype = MOTOR_COMMAND_LENGTH;
            }
            break;

        case ENCODER_REQUEST:
            if (!msgtype_flag && data == ENCODER_REQUEST) {
                uc_ptr->Rx_buffer[0] = data;
                uc_ptr->Rx_buflen++;
                msgtype_flag = 1;
                sendToMotorPIC_flag = 1;
                msgtype = ENCODER_LENGTH;
            }
            break;

        case ACK_LENGTH:
            if (msgtype_flag && data == ACK_LENGTH) {
                // Move message state.
                uc_ptr->Rx_buffer[uc_ptr->Rx_buflen] = data;
                uc_ptr->Rx_buflen++;
                msgtype = MESSAGE;
            } else {
                // Move back to original command ACK state.
                msgtype = COMMAND_ACK;
                msgtype_flag = 0;
            }
            break;


        case NACK_LENGTH:
            if (msgtype_flag && data == NACK_LENGTH) {
                // Move to message state
                uc_ptr->Rx_buffer[uc_ptr->Rx_buflen] = data;
                uc_ptr->Rx_buflen++;
                msgtype = MESSAGE;
            } else {
                // Move back to original NACK state.
                msgtype = COMMAND_NACK;
                msgtype_flag = 0;
            }
            break;

        case ENCODER_LENGTH:
            if (msgtype_flag && sendToMotorPIC_flag) {
                // Move to Check Sum State.
                uc_ptr->Rx_buffer[uc_ptr->Rx_buflen] = data;
                uc_ptr->Rx_buflen++;
                msg_flag = 1;
                msgtype = CHECKSUM;
            } else {
                // Move back to original encoder request state.
                msgtype = ENCODER_REQUEST;
                msgtype_flag = 0;
            }
            break;

        case SENSOR_LENGTH:
            if (msgtype_flag && sendToSensorPIC_flag) {
                // Move to check sum state.
                uc_ptr->Rx_buffer[uc_ptr->Rx_buflen] = data;
                uc_ptr->Rx_buflen++;
                msg_flag = 1;
                msgtype = CHECKSUM;
            } else {
                // Move back to orignal sensor request state.
                msgtype = SENSOR_REQUEST;
                msgtype_flag = 0;
            }
            break;

        case MOTOR_COMMAND_LENGTH:
            if (msgtype_flag && sendToMotorPIC_flag) {
                uc_ptr->Rx_buffer[uc_ptr->Rx_buflen] = data;
                uc_ptr->Rx_buflen++;
                msgtype = MESSAGE;
            } else {
                // Move back to original motor command state.
                msgtype = MOTOR_COMMAND;
                msgtype_flag = 0;
            }
            break;

        case MESSAGE:
            if (uc_ptr->Rx_buflen == uc_ptr->Rx_buffer[1] + 1) {
                uc_ptr->Rx_buffer[uc_ptr->Rx_buflen] = data;
                uc_ptr->Rx_buflen++;
                msg_flag = 1;
                msgtype = CHECKSUM;
            } else {
                uc_ptr->Rx_buffer[uc_ptr->Rx_buflen] = data;
                uc_ptr->Rx_buflen++;
                msgtype = MESSAGE;
            }
            break;

        case CHECKSUM:
            if (msg_flag) {
                uc_ptr->Rx_buffer[uc_ptr->Rx_buflen] = data;
                uc_ptr->Rx_buflen++;

                // Check for valid message
                unsigned char checkSum = 0;
                unsigned char bufLength = uc_ptr->Rx_buffer[1];

                if (bufLength == 0 && (uc_ptr->Rx_buffer[0] == MASTER_PIC || uc_ptr->Rx_buffer[0] == SENSOR_REQUEST || uc_ptr->Rx_buffer[0] == ENCODER_REQUEST)) {
                    // Check for sensor requests.
                    checkSum = 0x00;
                } else {
                    // Check if the entire message was passsed correctly with checksum.F
                    int i = 0;
                    for (; i < uc_ptr->Rx_buffer[1] + 2; i++) {
                        checkSum ^= uc_ptr->Rx_buffer[i];
                    }
                }

                // Send to a slave PIC or respond to ARM.
                if (checkSum != uc_ptr->Rx_buffer[uc_ptr->Rx_buflen] || uc_ptr->Rx_buffer[0] == COMMAND_NACK) {
                    // TODO Respond to NACK
                    // Send request to ARM slave to resend message.

                } else if (sendToSensorPIC_flag && uc_ptr->Rx_buffer[0] == SENSOR_REQUEST) {
                    // TODO Send to Sensor PIC using I2C2

                    //                    // Send sensor request to Sensor PIC.
                    //                    ToMainHigh_sendmsg(uc_ptr->Rx_buflen, MSGT_I2C_SEND, (void *) uc_ptr->Rx_buffer);
                } else if (sendToMotorPIC_flag && (uc_ptr->Rx_buffer[0] == MOTOR_COMMAND || uc_ptr->Rx_buffer[0] == ENCODER_REQUEST)) {
                    // Send Motor Command or Encoder Request to Motor Controller PIC.
                    ToMainHigh_sendmsg(uc_ptr->Rx_buflen, MSGT_MOTOR_SEND, (void *) uc_ptr->Rx_buffer);
                } else if (uc_ptr->Rx_buffer[0] == MASTER_PIC) {
                    //TODO use or get rid of.
                    uc_ptr->Rx_buflen = 0;
                }
            }
            // Reset the message flags.
            msgtype_flag = 0;
            sendToSensorPIC_flag = 0;
            sendToMotorPIC_flag = 0;
            uc_ptr->Rx_buflen = 0;
            msg_flag = 0;
            break;

        default:
            break;
    }

}