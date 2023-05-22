#include "MT6701_SPI.h"

void Read_Angle(foc_data *motor)
{
    static uint8_t data_RX[3] ;
    static uint32_t data ;
    static float angle;
    static float angle_el;

    HAL_GPIO_WritePin(SPI_CSN_GPIO_Port,SPI_CSN_Pin,GPIO_PIN_RESET);
    HAL_SPI_Receive(&hspi1,(uint8_t *)&data_RX[0],1,1000);
    HAL_SPI_Receive(&hspi1,(uint8_t *)&data_RX[1],1,1000);
    //HAL_SPI_Receive(&hspi1,(uint8_t *)&data_RX[2],1,1000);
    HAL_GPIO_WritePin(SPI_CSN_GPIO_Port,SPI_CSN_Pin,GPIO_PIN_SET);

    data = (data_RX[0]<<16)|(data_RX[1]<<8) | data_RX[2];
    data = (data>>10);
    angle=((float)data*360.0f)/16384.0f-motor->angle_Offect;
    while(angle<0) angle+=360.0f;
    angle=360-angle;
    angle_el=angle*(float)motor->pole_pairs;
    while(angle_el>360) angle_el-=360.0f;
    motor->angle=angle;
    motor->angle_el=angle_el;
    motor->angle_eRadian=angle_el*(float)FOC_EANGLE_TO_ERADIN;
}

void GetMotor_Speed(foc_data *motor)
{
    static float mangle = 0;
    static float last_mangle = 0;
    static float last_speed = 0;
    mangle = motor->angle;
    motor->speed = (mangle - last_mangle)*10;
    if(motor->speed > 1000 || motor->speed < -1000) motor->speed = last_speed;
    last_mangle = mangle;
    last_speed = motor->speed;
}