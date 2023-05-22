#ifndef _Motor_ADC_H__
#define _Motor_ADC_H__

#include "adc.h"
#include "BLDC_MOTOR.h"

extern short ADC_Value[2];
extern short adcValue[2];

void ADC_Init(void);
void GetMotorADC1PhaseCurrent(foc_data *motor);

#endif

