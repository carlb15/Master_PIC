/* 
 * File:   i2c_thread.h
 * Author: Carl
 *
 * Created on March 3, 2014, 8:33 PM
 */

#ifndef I2C_THREAD_H
#define	I2C_THREAD_H

typedef struct __i2c_thread_struct {
    // "persistent" data for this "lthread" would go here
    int data;
} i2c_thread_struct;

unsigned char sent_i2c_msg = 0x0;

int i2c_lthread(i2c_thread_struct *, int, int, unsigned char*);


#endif	/* I2C_THREAD_H */

