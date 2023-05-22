#ifndef _BLDC_MOTOR_H__
#define _BLDC_MOTOR_H__

#include "stdint-gcc.h"
#include <math.h>
#include "retarget.h"
#include "tim.h"

#define tpwm 7200.0f

struct _PI
{
    float kp;
    float ki;

    float pre;
    float tar;
    float bias;
    float lastBias;
    float out;
    float outMax;
};

struct _PID
{
    float kp;
    float ki;
    float kd;

    float pre;
    float tar;
    float bias;
    float lastBias;
    float out;
    float outMax;
    float err;
};

typedef struct FOC_DATA{
    int pole_pairs;
    float angle;
    float tar_angle;
    float angle_el;
    float angle_eRadian;//电角度弧度值
    float angle_Offect;//零偏角度
    float speed;
    float tar_speed;
    int sector;
    float iq_ref;
    float id_ref;
    float u_alpha;
    float u_beta;
    int ccr1;
    int ccr2;
    int ccr3;
    float ia;
    float ib;
    float ic;
    float i_alpha;
    float i_beta;
    float iq;
    float id;
    struct _PI idPID;
    struct _PI iqPID;
    struct _PI sPID;
    struct _PID pPID;
}foc_data;
extern foc_data motor_a;

typedef enum
{
    CW      = 1,  //clockwise顺时针
    CCW     = -1, // counter clockwise逆时针
    UNKNOWN = 0   //not yet known or invalid state
} Direction;

void brushless_motor_Init(void);
void setPhaseVoltage( foc_data *motor);
void FOC_SVPWM(foc_data *motor);
void Motor1AngleCalibration(void);

#endif
