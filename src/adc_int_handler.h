/* 
 * File:   adc_int_handler.h
 * Author: Miguel S
 *
 * Created on February 5, 2014, 4:40 PM
 */

#ifndef ADC_INT_HANDLER_H
#define	ADC_INT_HANDLER_H

#include "maindefs.h"



//unsigned char voltage;
float voltage;
unsigned int ADCResult;
int bufferLen = 0;
unsigned int previousValue =0;
//unsigned char buffer[20]; // buffer that stores all the readings
  unsigned char ADCBuffer0[I2C_MESSAGE_LENGTH];
  unsigned char ADCBuffer1[I2C_MESSAGE_LENGTH];

void adc_int_hand(void);
void start_adc(void);

#endif	/* ADC_INT_HANDLER_H */

