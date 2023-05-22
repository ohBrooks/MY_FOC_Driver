#ifndef _FOC_Math_H__
#define _FOC_Math_H__

#include "IQmathLib.h"
#include "BLDC_MOTOR.h"
#include <math.h>

#define _SQRT3 1.73205080757f
#define _SQRT3_2 0.86602540378f
#define _2PI 6.28318530718f
#define FOC_EANGLE_TO_ERADIN (0.017453293)  //角度值转化弧度制系数

extern double Position_Ref;

void Clarke(foc_data *motor);
void Park(foc_data *motor);
void Inv_Park(foc_data *motor);
void CurrentPIControlID(foc_data *motor);
void CurrentPIControlIQ(foc_data *motor);
void FocPID_Init(foc_data *motor);
void SpeedPIControlIQ(foc_data *motor);
void PositionPIDControl(foc_data *motor);

#endif

