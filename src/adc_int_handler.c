#include "maindefs.h"
#ifndef __XC8
#include <adc.h>
#else
#include <plib/adc.h>
#endif
#include "adc_int_handler.h"
#include "messages.h"
#include "debug.h"
#include "my_i2c.h"

void adc_int_hand() {


    //DEBUG_ON(ADC_ISR);
    //DEBUG_OFF(ADC_ISR);

    ADCResult = 0;
    ADCResult = (int) ReadADC() >> 2;

    if (previousValue != (unsigned int) ADCResult) {

        if (bufferFlag == 0x0) {
            ADCBuffer0[bufferLen++] = ADCResult;
            //
            if (bufferLen > I2C_MESSAGE_LENGTH) {
                int c = 0;
                for (; c < I2C_MESSAGE_LENGTH; c++) {
                    buff[c] = ADCBuffer0[c];
                }

                bufferLen = 0;
                bufferFlag = 0x1;
            }
        } else if (bufferFlag == 0x1) {
            ADCBuffer1[bufferLen++] = ADCResult;
            //
            if (bufferLen > I2C_MESSAGE_LENGTH) {
                int c = 0;
                for (; c < I2C_MESSAGE_LENGTH; c++) {
                    buff[c] = ADCBuffer0[c];
                }

                bufferLen = 0;
                bufferFlag = 0x0;
            }
        }

        previousValue = ADCResult;
    }
}

void start_adc(void) {

    ADC_INT_ENABLE();

    IPR1bits.ADIP = 1;
    ADCON0bits.GO = 0x1;
}
