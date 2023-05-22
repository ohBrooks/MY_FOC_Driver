#ifndef _MT6701_SPI_H__
#define _MT6701_SPI_H__

#include "IQmathLib.h"
#include <math.h>
#include "BLDC_MOTOR.h"
#include "spi.h"
#include "FOC_Math.h"

void Read_Angle(foc_data *motor);
void GetMotor_Speed(foc_data *motor);

#endif

