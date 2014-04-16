/* 
 * File:   sensor_thread.h
 * Author: ECE4534
 *
 * Created on April 13, 2014, 9:13 PM
 */

#ifndef SENSOR_THREAD_H
#define	SENSOR_THREAD_H

#define MAXSENSORBUF_TX 3
#define MAXSENSORBUF_RX 6

typedef struct __sensor_thread_struct {
    // "persistent" data for this "lthread" would go here
    int data;
    unsigned char Tx_buffer[MAXSENSORBUF_TX];
    unsigned char Rx_buffer[MAXSENSORBUF_RX];
} sensor_thread_struct;


void init_sensor_lthread(sensor_thread_struct*);
int sensor_lthread(sensor_thread_struct *, int, int, unsigned char*, unsigned char);


#endif	/* SENSOR_THREAD_H */

