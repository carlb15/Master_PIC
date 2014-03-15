/*
 * File:   debug.h
 * Author: ECE4534
 *
 * Created on February 10, 2014, 8:08 PM
 */

#ifndef DEBUG_H
#define DEBUG_H



// Define this to turn on instrumentation
#define DO_DEBUG
//      Use definitions to make debugging configurable

#define I2C_SEND LATBbits.LATB1
#define LENGTH_DEBUG LATBbits.LATB2
#define CHECKSUM_DEBUG LATBbits.LATB3

//      Define our debugging functions
#ifdef  DO_DEBUG

#define DEBUG_ON(a){ \
    a = 1; \
    }

#define DEBUG_OFF(a){ \
    a = 0; \
    }

#else
}
#define DEBUG_ON(a)
#define DEBUG_OFF(a)
#endif
#endif