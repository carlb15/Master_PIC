#ifndef __maindefs
#define __maindefs

#ifdef __XC8
#include <xc.h>
#ifdef _18F45J10
#define __USE18F45J10 1
#else
#ifdef _18F2680
#define __USE18F2680 1
#else
#ifdef _18F26J50
#define __USE18F26J50 
#else
#ifdef _18F46J50
#define __USE18F46J50 1
#endif
#endif
#endif
#endif
#else
#ifdef __18F45J10
#define __USE18F45J10 1
#else
#ifdef __18F2680
#define __USE18F2680 1
#else
#ifdef __18F26J50
#define __USE18F26J50 1
#else
#ifdef __18F46J50
#define __USE18F46J50 1
#endif
#endif
#endif
#endif
#include <p18cxxx.h>
#endif

// Message type definitions
#define MSGT_TIMER0 10
#define MSGT_TIMER1 11
#define MSGT_MAIN1 20
#define	MSGT_OVERRUN 30
#define MSGT_SEND_UART_DATA 31
#define MSGT_UART_DATA 32
#define MSGT_ARM_SEND 33
#define MSGT_ARM_RCV 34
#define MSGT_MOTOR_RCV 39
#define	MSGT_MOTOR_SEND 40
#define MSGT_SENSOR_SEND 41
#define MSGT_SENSOR_RCV 42


#define I2C_MESSAGE_LENGTH 5







#define MOTOR_COMMAND 0x01
#define SENSOR_ON_TRACK 0x02
#define SENSOR_OUT_OF_RANGE 0x03
#define STOP 0x05
#define MASTER_PIC 0x06
#define MOTOR_ENCODER 0x07
#define ALIGNMENT 0x08
#define NO_ENCODER_DATA 0x09
#define COMMAND_ACK 0x10
#define COMMAND_NACK 0x11
#define SENSOR_REQUEST 0x12
#define ENCODER_REQUEST 0x13

// State machine values
#define ACK_AND_NACK_LENGTH 0x14
#define SENSOR_LENGTH 0x15
#define MOTOR_COMMAND_LENGTH 0x16
#define MESSAGE 0x17
#define CHECKSUM 0x18
#define STOP_LENGTH 0x19
#endif