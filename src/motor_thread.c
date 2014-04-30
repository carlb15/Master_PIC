#include "maindefs.h"
#include <stdio.h>
#include "motor_thread.h"
#include "messages.h"
#include "debug.h"
#include "my_i2c.h"

// This is a "logical" thread that processes messages from the I2C
// It is not a "real" thread because there is only the single main thread
// of execution on the PIC because we are not using an RTOS.

int motor_thread(motor_thread_struct *motorptr, int msgtype, int length, unsigned char* msgbuffer, unsigned char slave_address) {

    if (msgtype == MSGT_MOTOR_SEND) {

        // Send a command to the motorcontroller
        if (i2c_master_send(length, msgbuffer, slave_address) == 0) {
            ToMainHigh_sendmsg(length, msgtype, (void *) msgbuffer);
        } else {
            // Retrieve data from the Motorcontroller PIC.
            ToMainHigh_sendmsg(length, MSGT_MOTOR_RCV, (void *) msgbuffer);
        }
    } else if (msgtype == MSGT_MOTOR_RCV) {

        // Change message length for encoder message.
        if (msgbuffer[0] == ENCODER_REQUEST) {
            length = 5;
        } else {
            length = 4;
        }

        if (i2c_master_recv(length, slave_address) == 0) {
            ToMainHigh_sendmsg(length, msgtype, (void *) msgbuffer);
        }
    }
}
