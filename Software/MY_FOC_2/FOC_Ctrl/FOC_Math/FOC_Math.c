#include "FOC_Math.h"

double Position_Ref=0;

void Clarke(foc_data *motor) //等幅值Clarke变换
{
    motor->i_alpha = motor_a.ia;
    motor->i_beta  = (motor_a.ia+2*motor_a.ib)/_SQRT3;
}

void Park(foc_data *motor)
{
    motor->id = motor->i_alpha*_IQ28toF(_IQ28cos(_IQ28(motor->angle_eRadian)))  +  motor->i_beta*_IQ28toF(_IQ28sin(_IQ28(motor->angle_eRadian)));
    motor->iq = -motor->i_alpha*_IQ28toF(_IQ28sin(_IQ28(motor->angle_eRadian)))  +  motor->i_beta*_IQ28toF(_IQ28cos(_IQ28(motor->angle_eRadian)));
}

void Inv_Park(foc_data *motor) //矩阵逆运算
{
    motor_a.u_alpha=_IQ28toF(_IQ28mpy(_IQ28cos(_IQ28(motor->angle_eRadian)),_IQ28(motor_a.idPID.out)) - _IQ28mpy(_IQ28sin(_IQ28(motor->angle_eRadian)),_IQ28(motor_a.iqPID.out)));
    motor_a.u_beta=_IQ28toF(_IQ28mpy(_IQ28sin(_IQ28(motor->angle_eRadian)),_IQ28(motor_a.idPID.out)) + _IQ28mpy(_IQ28cos(_IQ28(motor->angle_eRadian)),_IQ28(motor_a.iqPID.out)));
}

void CurrentPIControlID(foc_data *motor)
{
    //获取实际值
    motor->idPID.pre = motor->id;
    //获取目标值
    motor->idPID.tar = motor->id_ref;
    //计算偏差
    motor->idPID.bias = motor->idPID.tar - motor->idPID.pre;
    //计算PID输出值
    motor->idPID.out += _IQtoF(_IQmpy(_IQ(motor->idPID.kp), _IQ(motor->idPID.bias - motor->idPID.lastBias))) + _IQtoF(_IQmpy(_IQ(motor->idPID.ki), _IQ(motor->idPID.bias)));
    //保存偏差
    motor->idPID.lastBias = motor->idPID.bias;

    if (motor->idPID.out > (motor->idPID.outMax)) {
        motor->idPID.out = (motor->idPID.outMax);
    }

    if (motor->idPID.out < -(motor->idPID.outMax)) {
        motor->idPID.out = -(motor->idPID.outMax);
    }
}

void CurrentPIControlIQ(foc_data *motor)
{
    //获取实际值
    motor->iqPID.pre = motor->iq;
    //获取目标值
    motor->iqPID.tar = motor->iq_ref;
    //计算偏差
    motor->iqPID.bias = motor->iqPID.tar - motor->iqPID.pre;
    //计算PID输出值
    motor->iqPID.out += _IQtoF(_IQmpy(_IQ(motor->iqPID.kp), _IQ(motor->iqPID.bias - motor->iqPID.lastBias))) + _IQtoF(_IQmpy(_IQ(motor->iqPID.ki), _IQ(motor->iqPID.bias)));
    //保存偏差
    motor->iqPID.lastBias = motor->iqPID.bias;

    if (motor->iqPID.out > (motor->iqPID.outMax)) {
        motor->iqPID.out = (motor->iqPID.outMax);
    }

    if (motor->iqPID.out < -(motor->iqPID.outMax)) {
        motor->iqPID.out = -(motor->iqPID.outMax);
    }
}

void SpeedPIControlIQ(foc_data *motor)
{
    //获取实际值
    motor->sPID.pre = motor->speed;
    //获取目标值
    motor->sPID.tar = motor->tar_speed;
    //计算偏差
    motor->sPID.bias = motor->sPID.tar - motor->sPID.pre;
    //计算PID输出值
    motor->sPID.out += _IQtoF(_IQmpy(_IQ(motor->sPID.kp), _IQ(motor->sPID.bias - motor->sPID.lastBias))) + _IQtoF(_IQmpy(_IQ(motor->sPID.ki), _IQ(motor->sPID.bias)));
    //保存偏差
    motor->sPID.lastBias = motor->sPID.bias;

    if (motor->sPID.out > (motor->sPID.outMax)) {
        motor->sPID.out = (motor->sPID.outMax);
    }

    if (motor->sPID.out < -(motor->sPID.outMax)) {
        motor->sPID.out = -(motor->sPID.outMax);
    }
    motor->iq_ref = motor->sPID.out;
    motor->id_ref = 0;
}

void PositionPIDControl(foc_data *motor)
{
    float pre;
    motor->pPID.pre = motor->angle;   //获取当前位置
    motor->pPID.tar = motor->tar_angle; //获取目标位置

    //旋转坐标系
    pre = motor->pPID.tar - motor->pPID.pre;
    if(pre > (180.0f)) pre -= (360.0f);
    else if(pre < (-180.0f))  pre += (360.0f);

    motor->pPID.bias = pre;
    motor->pPID.out  = _IQ19toF(_IQ19mpy(_IQ19(motor->pPID.kp) , _IQ19(motor->pPID.bias))) + _IQ19toF(_IQ19mpy(_IQ19(motor->pPID.kd),_IQ19(motor->pPID.bias - motor->pPID.lastBias))) +  _IQ19toF(_IQ19mpy(_IQ19(motor->pPID.ki) , _IQ19(motor->pPID.err)));
    motor->pPID.lastBias = motor->pPID.bias;
    motor->pPID.err  += motor->pPID.bias;

    if (motor->pPID.out > (motor->pPID.outMax)) {
        motor->pPID.out = (motor->pPID.outMax);
    } else if (motor->pPID.out < -(motor->pPID.outMax)) {
        motor->pPID.out = -(motor->pPID.outMax);
    }
    motor_a.tar_speed = motor->pPID.out;
}

void FocPID_Init(foc_data *motor)
{
    motor->idPID.outMax=1.0f;
    motor->idPID.out=0;
    motor->idPID.bias=0;
    motor->idPID.lastBias=0;
    motor->idPID.pre=0;
    motor->idPID.tar=0;
    motor->idPID.ki=0.08f;
    motor->idPID.kp=0.04f;

    motor->iqPID.outMax=3.0f;//6.9f
    motor->iqPID.out=0;
    motor->iqPID.bias=0;
    motor->iqPID.lastBias=0;
    motor->iqPID.pre=0;
    motor->iqPID.tar=0;
    motor->iqPID.ki=0.08f;
    motor->iqPID.kp=0.08f;

    motor->sPID.outMax=3.0f;
    motor->sPID.out=0;
    motor->sPID.bias=0;
    motor->sPID.lastBias=0;
    motor->sPID.pre=0;
    motor->sPID.tar=0;
    motor->sPID.ki=-0.018f;
    motor->sPID.kp=-0.9f;

    motor->pPID.err=0.0f;
    motor->pPID.outMax=4.0f;
    motor->pPID.out=0;
    motor->pPID.bias=0;
    motor->pPID.lastBias=0;
    motor->pPID.pre=0;
    motor->pPID.tar=0;
    motor->pPID.kd=2.2f;//2.2f
    motor->pPID.ki=0.0f;
    motor->pPID.kp=0.2f;//0.2
}