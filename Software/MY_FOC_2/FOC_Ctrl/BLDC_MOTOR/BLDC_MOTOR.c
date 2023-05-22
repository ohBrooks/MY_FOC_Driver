#include "BLDC_MOTOR.h"
#include "FOC_Math.h"
#include "MT6701_SPI.h"

void brushless_motor_Init(void)
{
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
    HAL_GPIO_WritePin(EN_GPIO_Port,EN_Pin,GPIO_PIN_SET);
}

void setPwm(uint32_t ccr1, uint32_t ccr2, uint32_t ccr3)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr3);
}

/*编码器角度校准*/
void Motor1AngleCalibration(void)
{
    float angle;
    //1.电机旋转至a轴
    setPwm(2520, 720, 720);
    HAL_Delay(500);
    //2.读取角度
    for (uint8_t i = 0; i < 10; i++) {
        Read_Angle(&motor_a);
        printf("1:float%f\r\n",motor_a.angle);
        HAL_Delay(100);
    }
}

//angle_el：目标角度
void setPhaseVoltage(foc_data *motor)
{
    uint8_t N=0;
    //确定所在扇区
    float A=motor->u_beta;
    float B=_IQ28toF(_IQ28mpy(_IQ28(_SQRT3_2),_IQ28(motor->u_alpha)) - _IQ28mpy(_IQ28(0.5f),_IQ28(motor->u_beta)));
    float C=_IQ28toF(_IQ28mpy(_IQ28(-_SQRT3_2),_IQ28(motor->u_alpha)) - _IQ28mpy(_IQ28(0.5f),_IQ28(motor->u_beta)));

    if(A>0)
        N=N+1;

    if(B>0)
        N=N+2;

    if(C>0)
        N=N+4;

    //float X=(_SQRT3*tpwm)/12.0f*A;
    //float Y=(_SQRT3)*(tpwm)/(12.0f)*(-C);
    //float Z=(_SQRT3)*(tpwm)/(12.0f)*(-B);
    float X=_IQ17toF(_IQ17mpy(_IQ17div(_IQ17mpy(_IQ17(_SQRT3),_IQ17(tpwm)),_IQ17(12.0f)),_IQ17(A)));
    float Y=_IQ17toF(_IQ17mpy(_IQ17div(_IQ17mpy(_IQ17(_SQRT3),_IQ17(tpwm)),_IQ17(12.0f)),_IQ17(-C)));
    float Z=_IQ17toF(_IQ17mpy(_IQ17div(_IQ17mpy(_IQ17(_SQRT3),_IQ17(tpwm)),_IQ17(12.0f)),_IQ17(-B)));


    // calculate the duty cycles 计算占空比
    float Ta,Tb;
    switch(N){
        case 3:
            Ta = -Z;
            Tb = X;
            break;
        case 1:
            Ta = Z;
            Tb = Y;
            break;
        case 5:
            Ta = X;
            Tb = -Y;
            break;
        case 4:
            Ta = -X;
            Tb = Z;
            break;
        case 6:
            Ta = -Y;
            Tb = -Z;
            break;
        case 2:
            Ta = Y;
            Tb = -X;
            break;
        default:
            // possible error state
            Ta = 0;
            Tb = 0;
    }

    float Value1= _IQ17toF(_IQ17div(_IQ17(tpwm - Ta - Tb),_IQ17(4.0f)));
    float Value2= Value1 + _IQ17toF(_IQ17div(_IQ17(Ta),_IQ17(2.0f)));
    float Value3= Value2 + _IQ17toF(_IQ17div(_IQ17(Tb),_IQ17(2.0f)));

    switch(N){
        case 3:
            motor->ccr1=(int)Value1;
            motor->ccr2=(int)Value2;
            motor->ccr3=(int)Value3;
            break;
        case 1:
            motor->ccr1=(int)Value2;
            motor->ccr2=(int)Value1;
            motor->ccr3=(int)Value3;
            break;
        case 5:
            motor->ccr1=(int)Value3;
            motor->ccr2=(int)Value1;
            motor->ccr3=(int)Value2;
            break;
        case 4:
            motor->ccr1=(int)Value3;
            motor->ccr2=(int)Value2;
            motor->ccr3=(int)Value1;
            break;
        case 6:
            motor->ccr1=(int)Value2;
            motor->ccr2=(int)Value3;
            motor->ccr3=(int)Value1;
            break;
        case 2:
            motor->ccr1=(int)Value1;
            motor->ccr2=(int)Value3;
            motor->ccr3=(int)Value2;
            break;
        default:
            // possible error state
            motor->ccr1=0;
            motor->ccr2=0;
            motor->ccr3=0;
    }

    // set the voltages in driver
    setPwm(motor->ccr1, motor->ccr2, motor->ccr3);
}

void FOC_SVPWM(foc_data *motor)
{
    Inv_Park(motor);
    setPhaseVoltage(motor);
}


