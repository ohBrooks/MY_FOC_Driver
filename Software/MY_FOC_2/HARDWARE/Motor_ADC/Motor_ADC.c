#include "Motor_ADC.h"

short ADC_Value[2]={0,0};
short adcValue[2]={0,0};

void ADC_Init(void)
{
    HAL_ADCEx_Calibration_Start(&hadc1);    //AD校准
    HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&ADC_Value,2); //DMA
}

void GetMotorADC1PhaseCurrent(foc_data *motor)
{
    adcValue[0] = (short) (ADC_Value[0] - 2048);
    adcValue[1] = (short) (2048 - ADC_Value[1]);
    motor->ic = (float) adcValue[0] * 0.001611f * 1.5f;
    motor->ia = (float) adcValue[1] * 0.001611f * 1.5f;
    motor->ib = 0 - motor->ic - motor->ia;
}