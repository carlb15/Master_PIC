#include "maindefs.h"
#ifndef __XC8
#include <plib/i2c.h>
#include <i2c.h>
#include <timer0_thread.h>
#include <stdio.h>
#include<string.h>
#else
#include <plib/i2c.h>
#endif
#include "my_i2c.h"
#include "debug.h"
#include <string.h>

static i2c_comm *ic_ptr;
//  char c;
int length;
unsigned char msgtype;
unsigned char last_reg_recvd;
unsigned char msgbuffer[20];
/*
 A 100kHz I2C bus for master
 * 1. The microcontroller 48 MHz.
 *  SSPADD = (48000000 / (4*100000))-1; //for a 48 MHz clock
 *
 */
#define I2C_100KHZ 0x77 // 28 before

#define I2C_ENABLE_SLEW 0b00000000 // Sets SMP to 0, which is enabled
#define I2C_DISABLE_SLEW 0b10000000 // Sets SMP to 1, which is disabled
#define I2C_ENABLE_SMBUS 0b01000000 // Sets CKE to 1, which is enabled
#define I2C_DISABLE_SMBUS 0b00000000 // Sets CKE to 0, which is disabled
#define I2C_SCL PORTBbits.SCL1
#define I2C_SDA PORTBbits.SDA1

/*
 * SSPCON1
 *
 * The following are bits that should be bitwise or'd
 * to set certain conditions for I2C.
 */
#define I2C_MASTER_MODE 0b00001000
#define I2C_ENABLE 0b00100000 // Sets SSPEN to 1, which is enabled

//uart_thread_struct uthread_data; // info for uart_lthread
//timer1_thread_struct t1thread_data; // info for timer1_lthread
//timer0_thread_struct t0thread_data; // info for timer0_lthread
// Configure for I2C Master mode -- the variable "slave_addr" should be stored in
//   i2c_comm (as pointed to by ic_ptr) for later use.

void i2c_configure_master() {
    //ic_ptr->slave_addr = slave_addr;
    //no overflow,no collision, SSPEN =1, 4 =? , 1000 as master
    SSPCON1 = I2C_MASTER_MODE;
    // Enable I2C
    SSPCON1 |= I2C_ENABLE;
    //BRG 100KHz
    SSPADD = I2C_100KHZ;
    //counter to keep track which byte we are sending
    //  ic_ptr->bufferCounterSend = 1;
}

// Sending in I2C Master mode [slave write]
//              returns -1 if the i2c bus is busy
//              return 0 otherwise
// Will start the sending of an i2c message -- interrupt handler will take care of
//   completing the message send.  When the i2c message is sent (or the send has failed)
//   the interrupt handler will send an internal_message of type MSGT_MASTER_SEND_COMPLETE if
//   the send was successful and an internal_message of type MSGT_MASTER_SEND_FAILED if the
//   send failed (e.g., if the slave did not acknowledge).  Both of these internal_messages
//   will have a length of 0.
// The subroutine must copy the msg to be sent from the "msg" parameter below into
//   the structure to which ic_ptr points [there is already a suitable buffer there].

unsigned char i2c_master_send(unsigned char length, unsigned char *msg, unsigned char slave_address) {

    ic_ptr->slave_addr = slave_address;

    //check that bus is not currently transmitting
    if (ic_ptr->status != I2C_IDLE_) {
        return 0;
    }
    // 0 for R/W bit = Transmitter ... to 9E
    //ic_ptr->slave_addr = (ic_ptr->slave_addr << 1) & 0xFE;
    //ic_ptr->slave_addr = (((ic_ptr->slave_addr) >> 1) & 0xAF);
    //first byte of out = address
    //setting the counter send
    ic_ptr->bufferCounterSend = 1;
    ic_ptr->outbuffer[0] = ic_ptr->slave_addr;
    //add one to the length to include the address
    ic_ptr->outbuflen = length + 1;
    //copy message
    int i = 1;
    for (i; i < length + 1; i++) {
        ic_ptr->outbuffer[i] = msg[i - 1];
    }
    //setting master handler to send
    ic_ptr->tx_i2c = 0x1;
    ic_ptr->rx_i2c = 0x0;

    ic_ptr->status = I2C_START_COND;
    //Generate 1st start condition
    StartI2C(); // Start

    return 1;
}
// Receiving in I2C Master mode [slave read]
//              returns -1 if the i2c bus is busy
//              return 0 otherwise
// Will start the receiving of an i2c message -- interrupt handler will take care of
//   completing the i2c message receive.  When the receive is complete (or has failed)
//   the interrupt handler will send an internal_message of type MSGT_MASTER_RECV_COMPLETE if
//   the receive was successful and an internal_message of type MSGT_MASTER_RECV_FAILED if the
//   receive failed (e.g., if the slave did not acknowledge).  In the failure case
//   the internal_message will be of length 0.  In the successful case, the
//   internal_message will contain the message that was received [where the length
//   is determined by the parameter passed to i2c_master_recv()].
// The interrupt handler will be responsible for copying the message received into

unsigned char i2c_master_recv(unsigned char length, unsigned char slave_address) {

    ic_ptr->slave_addr = slave_address;

    //check that bus is not currently transmitting
    if (ic_ptr->status != I2C_IDLE_) {
        return 0;
    }
    //setting master handler to receive
    ic_ptr->rx_i2c = 0x1;
    ic_ptr->tx_i2c = 0x0;
    // 1 for R/W bit = WRITE ...CF to 9E
    ic_ptr->slave_addr_rc = ((ic_ptr->slave_addr) | 0b00000001);
    //getting lenght of data
    ic_ptr->buflen = length;
    //moving to next state for next interrupt
    ic_ptr->status = I2C_START_COND_REC;
    //initializing buffer
    ic_ptr->bufferCounterRx = 0;
    //Generate 1st start condition
    StartI2C(); // Start
    return 1;
}

void i2c_int_handler() {
    // check if we are receiving or transmitting
    if ((ic_ptr->rx_i2c == 1) && (ic_ptr->tx_i2c == 0)) {
        i2c_int_handler_master_rx();
    } else if ((ic_ptr->tx_i2c == 1) && (ic_ptr->rx_i2c == 0)) {
        i2c_int_handler_master_tx();
    }
}

void i2c_int_handler_master_rx() {

    switch (ic_ptr->status) {

        case I2C_START_COND_REC:
        {
            //loading the address after starting condition
            //load address in buffer
            SSPBUF = ic_ptr->slave_addr_rc;
            ic_ptr->status = I2C_ACK_ADD_SEND_REC;
            break;
        };

        case I2C_ACK_ADD_SEND_REC:
        {
            //state to see if slave received the address..
            //ack  received from slave
            if (!SSPCON2bits.ACKSTAT) {
                //master configured as a receiver
                SSPCON2bits.RCEN = 1;
                //send ACK in next status
                ic_ptr->status = I2C_SEND_ACK;
            } else {
                ic_ptr->status = I2C_ERR_NACK_ADD_REC;
            }
            break;
        };

        case I2C_SEND_ACK:
        {
            //read first byte
            ic_ptr->buffer[ic_ptr->bufferCounterRx] = SSPBUF;
            ic_ptr->bufferCounterRx++;
            if (ic_ptr->buflen == ic_ptr->bufferCounterRx) {
                SSPCON2bits.ACKDT = 1;
                SSPCON2bits.ACKEN = 1;
                ic_ptr->status = I2C_AFTER_ACKEN;
            } else {
                SSPCON2bits.ACKDT = 0;
                SSPCON2bits.ACKEN = 1;
                //now we are going to get another interrupt
                ic_ptr->status = I2C_AFTER_SEND_ACK;
            }
            break;
        };

        case I2C_AFTER_SEND_ACK:
        {
            // now we got an interrupt from the ack we sent
            //master configured as a receiver
            SSPCON2bits.RCEN = 1;
            //send ACK in next status
            ic_ptr->status = I2C_SEND_ACK;
            break;
        };
        case I2C_AFTER_ACKEN:
        {
            //sending stop bit
            SSPCON2bits.PEN = 1;
            ic_ptr->status = I2C_STOP_RX;
            break;
        };
        case I2C_STOP_RX:
        {
            ic_ptr->status = I2C_IDLE_;

            if (ic_ptr->buffer[0] == 0x03) {
                DEBUG_ON(SENSOR_DBG);
                DEBUG_OFF(SENSOR_DBG);
                // Send sensor out of range to ARM PIC
                ToMainLow_sendmsg(ic_ptr->bufferCounterRx, MSGT_ARM_SEND, (void *) ic_ptr->buffer);
            } else if (ic_ptr->buffer[0] == 0x05) {
                DEBUG_ON(SENSOR_DBG);
                DEBUG_OFF(SENSOR_DBG);

                DEBUG_ON(SENSOR_DBG);
                DEBUG_OFF(SENSOR_DBG);
                // Send Stop command to Motorcontroller PIC
                ToMainHigh_sendmsg(ic_ptr->bufferCounterRx, MSGT_MOTOR_SEND, (void *) ic_ptr->buffer);
            } else if (ic_ptr->buffer[0] == 0x07) {
                DEBUG_ON(SENSOR_DBG);
                DEBUG_OFF(SENSOR_DBG);

                DEBUG_ON(SENSOR_DBG);
                DEBUG_OFF(SENSOR_DBG);

                DEBUG_ON(SENSOR_DBG);
                DEBUG_OFF(SENSOR_DBG);
                // Buffer Encoder data
                ToMainLow_sendmsg(ic_ptr->bufferCounterRx, MSGT_ARM_SEND, (void *) ic_ptr->buffer);
            } else if (ic_ptr->buffer[0] == 0x08) {
                DEBUG_ON(SENSOR_DBG);
                DEBUG_OFF(SENSOR_DBG);

                DEBUG_ON(SENSOR_DBG);
                DEBUG_OFF(SENSOR_DBG);

                DEBUG_ON(SENSOR_DBG);
                DEBUG_OFF(SENSOR_DBG);

                DEBUG_ON(SENSOR_DBG);
                DEBUG_OFF(SENSOR_DBG);
                //  Send Alignment command to Motorcontroller PIC
                ToMainHigh_sendmsg(ic_ptr->bufferCounterRx, MSGT_MOTOR_SEND, (void *) ic_ptr->buffer);
            }
            break;
        };
        default:
        {
            break;
        };
    };
}

void i2c_int_handler_master_tx() {
    // Starting bit raised first flag.. so we are ready to send address first after
    // I2C is enabled.
    switch (ic_ptr->status) {

        case I2C_START_COND:
        {
            //loading the address
            SSPBUF = ic_ptr->outbuffer[ic_ptr->outbufind];
            ic_ptr->status = I2C_ACK_ADD_SEND;
            break;
        };

        case I2C_ACK_ADD_SEND:
        {
            //this case checks the ack from the address sent
            //ack  received from slave
            if (!SSPCON2bits.ACKSTAT) {
                //clear ack
                SSPCON2bits.ACKDT = 1;
                //load buffer with data depending on the position: poistion is 1 since 0 is the address
                SSPBUF = ic_ptr->outbuffer[ic_ptr->bufferCounterSend];
                //to send next element in buffer
                ic_ptr->bufferCounterSend++;
                ic_ptr->status = I2C_ACK_DATA_SEND;

                //ack not received
            } else {
                ic_ptr->status = I2C_ERR_NACK_ADD;
            }
            break;
        };

        case I2C_ACK_DATA_SEND:
        {
            //now we check if the data was sent
            //ack  received from slave
            if (!SSPCON2bits.ACKSTAT) {
                //clear flag
                //PIR1bits.SSPIF = 0;
                //clear ack
                SSPCON2bits.ACKDT = 1;
                //check if we sent the las byte of the package
                if (ic_ptr->bufferCounterSend == (ic_ptr->outbuflen)) {
                    ic_ptr->status = I2C_ACK_STOP;
                    //enable stop condition
                    SSP1CON2bits.PEN = 1;

                } else {
                    //load buffer with data depending on the position
                    SSPBUF = ic_ptr->outbuffer[ic_ptr->bufferCounterSend];
                    //to send next element in buffer
                    ic_ptr->bufferCounterSend++;
                    ic_ptr->status = I2C_ACK_DATA_SEND;
                }

            } else {
                ic_ptr->status = I2C_ERR_NACK_DATA;
            }
            break;
        };
        case I2C_ACK_STOP:
        {
            ic_ptr->status = I2C_IDLE_;
            break;
        };
        default:
        {
            break;
        };
    };
}



// set up the data structures for this i2c code
// should be called once before any i2c routines are called

void init_i2c(i2c_comm * ic) {
    ic_ptr = ic;
    ic_ptr->buflen = 0;
    ic_ptr->event_count = 0;
    ic_ptr->status = I2C_IDLE_;
    ic_ptr->error_count = 0;
    ic_ptr->outbufind = 0;
}