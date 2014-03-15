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
#define __USE18F26J50 1
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
#define MSGT_I2C_RCV 39
#define	MSGT_I2C_SEND 40
#define MSGT_I2C_RQST 42
//was 42
#define MSGT_I2C_MASTER_SEND_COMPLETE 43
#define MSGT_I2C_MASTER_SEND_FAILED 44
#define MSGT_I2C_MASTER_RECV_COMPLETE 45
#define MSGT_I2C_MASTER_RECV_FAILED 46
#define MSGT_ADC_DATA 50
#define I2C_MESSAGE_LENGTH 5
#define SENSOR_MSGTYPE 0xaa



#endif

