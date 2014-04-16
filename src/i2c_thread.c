#include "maindefs.h"
#include <stdio.h>
#include "i2c_thread.h"
#include "messages.h"
#include "debug.h"
#include "my_i2c.h"

// This is a "logical" thread that processes messages from the I2C
// It is not a "real" thread because there is only the single main thread
// of execution on the PIC because we are not using an RTOS.

int i2c_lthread(i2c_thread_struct *i2cptr, int msgtype, int length, unsigned char* msgbuffer) {

    if (msgtype == MSGT_I2C_SEND) {
<<<<<<< HEAD

        DEBUG_ON(I2C_SEND_DBG);
        DEBUG_OFF(I2C_SEND_DBG);

        unsigned char msgtype_moto = 0x01;

        msgForMotorcontroller(msgtype_moto, length, msgbuffer);

        // Send a command to the motorcontroller
=======
        // Send a motor command.
>>>>>>> 8b453ca2904bb6ddc1f44b84ffcd82707b0e4488
        if (i2c_master_send(length, msgbuffer) == 0) {
            ToMainHigh_sendmsg(length, msgtype, (void *) msgbuffer);
        } else {
            // Retrieve data from the Motorcontroller PIC.
            ToMainHigh_sendmsg(length, MSGT_I2C_RCV, (void *) msgbuffer);
        }
    } else if (msgtype == MSGT_I2C_RCV) {

        // Retrieve motor data.
        if (msgbuffer[0] == SENSOR_MSGTYPE) {
            length = 5;
        }

        if (i2c_master_recv(length) == 0) {
            ToMainHigh_sendmsg(length, msgtype, (void *) msgbuffer);
        }
    }
}
