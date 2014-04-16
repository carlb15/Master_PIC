#ifndef __my_uart_h
#define __my_uart_h

#include "messages.h"

#define MAXUARTBUF 10


#if (MAXUARTBUF > MSGLEN)
#define MAXUARTBUF MSGLEN
#endif

typedef struct __uart_comm {
    unsigned char Tx_buffer[MAXUARTBUF];
    unsigned char Rx_buffer[MAXUARTBUF];
    unsigned char Tx_buflen;
    unsigned char Rx_buflen;
    unsigned char msg_length;
} uart_comm;

unsigned char uartData;

unsigned char msgtype = 0;
unsigned char msgtype_flag = 0x0, length_flag = 0x0, msg_flag = 0x0, checksum_flag = 0x0;
unsigned char sendToSensorPIC_flag = 0x0, sendToMotorPIC_flag = 0x0;

void init_uart_recv(uart_comm *);
void uart_send_int_handler();
void uart_recv_int_handler(void);
void uart_retrieve_buffer(int length, unsigned char*);
void checkForValidMsgType(char data);
#endif