//
// Created by Trisoil on 2022/1/21.
//

#include <math.h>
#include "Move_Controlor.h"
#include "PID_Controlor.h"
#include "CAN_Operation.h"
#include "string.h"
#include "Usart_Printf.h"
#include "Location_Module_Recieve.h"
#include "main.h"

#define PI 3.1415926f
#define WHEEL_LEFT2RIGHT 40.0f //车辆长边
#define WHEEL_FRONT2BACK 40.0f //车辆短边
#define DRIVE_WHEEL_RADIUS 7.62f //轮子半径
/**
 * @brief 全向移动运动学
 * @param speed 速度大小(m/s)
 * @param dir 速度方向/rad
 * @param omega 自转角速度(rad/s)，逆时针为正方向
 * @note 根据速度矢量计算出四个轮的转速
 **/
void MecanumChassis_OmniDrive(float vx, float vy, float omega)
{
    memset(Current,0,sizeof(Current));

    float target_speed[5];   // 每个车轮的切向速度
    float a = WHEEL_LEFT2RIGHT / 2, b = WHEEL_FRONT2BACK / 2;

    //>>>直接计算法<<<
    target_speed[3] = vx + vy - omega * (a + b);
    target_speed[4] = -vx + vy + omega * (a + b);
    target_speed[2] = vx + vy + omega * (a + b);
    target_speed[1] = -vx + vy - omega * (a + b);
    for (int i = 1; i <= 4; i++)
    {
        float target_omega = target_speed[i] / DRIVE_WHEEL_RADIUS; // target_speed单位m/s
        float target_rpm = target_omega * 30.0f / PI;
        if (i % 2 == 0) target_rpm = -target_rpm;
        Base_Speed_Calculation(i, target_rpm);
    }
    CAN_CMD_Current(Current[1],Current[2],Current[3],Current[4]);
    HAL_Delay(10);
}

void Gyro_Mode_Drive(float vx,float vy, float omega)
{
	float Angle = PI*(Location_Data.Angle/180.0f);
	float vx_output = (float)(vy*sin((double)Angle)) + (float)(vx*cos((double)Angle));
	float vy_output = (float)(vy*cos((double)Angle)) - (float)(vx*sin((double)Angle));
	MecanumChassis_OmniDrive(-vx_output,vy_output,omega);
}