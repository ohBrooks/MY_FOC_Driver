#include "User_APP.h"

void User_APP(foc_data *motor,int mode)
{
    static int i=0;
    static float iqPID_Last_out=0;
    switch (mode) {
        case 1://速度变化
            motor->iqPID.outMax=6.9f;
            i++;
            if(i>=0xfff) {
                i=0;
                if(motor->tar_speed==-3.0f) motor->tar_speed=3.0f;
                else if(motor->tar_speed==3.0f) motor->tar_speed=-3.0f;
            }break;
        case 2://速度梯度变化
            motor->iqPID.outMax=6.9f;
            i++;
            if(i>=0x2ff) {
                i=0;
                if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET) {
                    motor->tar_speed += 0.1f;
                } else if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET) {
                    motor->tar_speed -= 0.1f;
                }
            }
            break;
        case 3://位置闭环
            motor->iqPID.outMax=2.0f;
            i++;
            if(i>=0x2ff) {
                i=0;
                if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET) {
                    motor->tar_angle += 45.0f;
                } else if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET) {
                    motor->tar_angle -= 45.0f;
                }
            }
            PositionPIDControl(motor);
            break;
        case 4://多档开关
            motor->iqPID.outMax=2.0f;
            if(motor->pPID.bias>=45.0f) motor->tar_angle -= 90.0f;
            else if(motor->pPID.bias<=-45.0f) motor->tar_angle += 90.0f;
            PositionPIDControl(motor);
            break;
        case 5://surface dial旋钮
            motor->iqPID.outMax=0.3f;
            break;
        case 6://失重旋转
            motor->iqPID.outMax=2.0f;
            if(motor->iqPID.out - iqPID_Last_out >=0.05 || motor->iqPID.out - iqPID_Last_out <=-0.05)
            {
                i=1;
            }
            if(i==1 )
            {
                i=0;
                motor->tar_speed=motor->speed;
            }
            iqPID_Last_out = motor->iqPID.out;
            break;
        case 7://限位开关
            if(motor->angle >= 90 && motor->angle<=270)
            {
                motor->iqPID.outMax=0.3f;
            }
            else if(motor->angle < 90 )
            {
                motor->iqPID.outMax=1.5f;
                motor->tar_angle = 90.0f;
                PositionPIDControl(motor);
            }
            else if(motor->angle > 270)
            {
                motor->iqPID.outMax=1.5f;
                motor->tar_angle = 270.0f;
                PositionPIDControl(motor);
            }
            break;
        default:
            break;
    }
}