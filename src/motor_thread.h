/* 
 * File:   motor_thread.h
 * Author: ECE4534
 *
 * Created on April 13, 2014, 9:11 PM
 */

#ifndef MOTOR_THREAD_H
#define	MOTOR_THREAD_H

typedef struct __i2c_thread_struct {
    // "persistent" data for this "lthread" would go here
    int data;
} motor_thread_struct;

unsigned char sent_i2c_msg = 0x0;

int motor_thread(motor_thread_struct *, int, int, unsigned char*, unsigned char);
#endif	/* MOTOR_THREAD_H */

