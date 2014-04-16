#include "maindefs.h"
#include <stdio.h>
#include "i2c_thread.h"
#include "messages.h"
#include "debug.h"
#include "my_i2c.h"
#include "sensor_thread.h"

static sensor_thread_struct* sensor_struct_data;

// This is a "logical" thread that processes messages from the I2C
// It is not a "real" thread because there is only the single main thread
// of execution on the PIC because we are not using an RTOS.

int sensor_lthread(sensor_thread_struct *sensorptr, int msgtype, int length, unsigned char* msgbuffer, unsigned char slave_address) {

    if (msgtype == MSGT_SENSOR_SEND) {

        unsigned char buf[3];
        buf[0] = 0x12;
        buf[1] = 0x00;
        buf[2] = 0x00;
        length = 3;

        if (i2c_master_send(length, buf, slave_address) == 0) {
            ToMainHigh_sendmsg(length, msgtype, (void *) msgbuffer);
        } else {
            // Retrieve data from the Motorcontroller PIC.
            ToMainHigh_sendmsg(length, MSGT_SENSOR_RCV, (void *) msgbuffer);
        }
    } else if (msgtype == MSGT_SENSOR_RCV) {

        //  Receive message from Sensor PIC and send to next PIC.
        if (i2c_master_recv(MAXSENSORBUF_RX, slave_address) == 0) {
            ToMainHigh_sendmsg(length, msgtype, (void *) msgbuffer);
        }
    }
}

void init_sensor_lthread(sensor_thread_struct* s) {
    sensor_struct_data = s;
    // Setup TX Buffer
    sensor_struct_data->Tx_buffer[0] = 0x12;
    sensor_struct_data->Tx_buffer[1] = 0x00;
    sensor_struct_data->Tx_buffer[2] = 0x00;
}