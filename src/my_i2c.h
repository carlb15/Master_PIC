#ifndef __my_i2c_h
#define __my_i2c_h

#include "messages.h"
#define MAXI2CBUF 10

typedef struct __i2c_comm {
    unsigned char buffer[MAXI2CBUF];
    unsigned char buflen;
    unsigned char event_count;
    unsigned char status;
    unsigned char error_code;
    unsigned char error_count;
    unsigned char outbuffer[MAXI2CBUF];
    unsigned char outbuflen;
    unsigned char outbufind;
    unsigned char slave_addr;
    int bufferCounterSend;
    int bufferCounterRx;
    unsigned char tx_i2c;
    unsigned char rx_i2c;
    unsigned char slave_addr_rc;
} i2c_comm;

int bufferFlag;
unsigned char buff[I2C_MESSAGE_LENGTH];

#define I2C_IDLE_ 0x3
#define I2C_START_COND 0x5
#define I2C_ACK_ADD_SEND 0x6
#define	I2C_ACK_DATA_SEND 0x7
#define I2C_STOP_COND 0x8
#define I2C_ERR_NACK_DATA 0x9
#define I2C_ACK_STOP 0xA
#define I2C_ERR_NACK_ADD 1

////////////////////////////
#define I2C_IDLE_REC 0xB
#define I2C_START_COND_REC 0xC
#define I2C_ACK_ADD_SEND_REC 0xD
#define I2C_ERR_NACK_ADD_REC 0xE
#define I2C_SEND_ACK 0xF
#define I2C_AFTER_SEND_ACK 0x10
#define I2C_AFTER_ACKEN 0x11
#define  I2C_STOP_RX 0x12





void init_i2c(i2c_comm *);
void i2c_int_handler_master_tx(void);
void i2c_int_handler_master_rx(void);
void i2c_int_handler(void);
void start_i2c_slave_reply(unsigned char, unsigned char *);
void i2c_configure_slave(unsigned char);
void i2c_configure_master(unsigned char);
unsigned char i2c_master_send(unsigned char, unsigned char *);
unsigned char i2c_master_recv(unsigned char);
void readMessages();
void i2c_start_communication(void);

#endif