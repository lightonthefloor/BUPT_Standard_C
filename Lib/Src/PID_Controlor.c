//
// Created by Trisoil on 2021/12/14.
//


#include <stdlib.h>
#include "PID_Controlor.h"
#include "CAN_Operation.h"
#include "Location_Module_Recieve.h"
#include "gpio.h"
#include "main.h"
#include "can.h"
#include "math.h"


PID_Number PID_Control_Num[8];
int16_t Current[5];

// 3.5 0.05 0.3
void M3508_Speed_Control_Init(uint8_t ID_Num){
    PID_Control_Num[ID_Num].kp = 3.5f;
    PID_Control_Num[ID_Num].ki = 0.05f;
    PID_Control_Num[ID_Num].kd = 0.3f;
    PID_Control_Num[ID_Num].max_output  = 16384;
    PID_Control_Num[ID_Num].integral_limit = 10000;
    PID_Control_Num[ID_Num].deadband = 10;
    PID_Control_Num[ID_Num].max_err = 8000;
}


int16_t PID_Calculate(float Set_Speed,uint8_t ID_Num){
    PID_Control_Num[ID_Num].ki_output += PID_Control_Num[ID_Num].ki * PID_Control_Num[ID_Num].now_err;
    if (PID_Control_Num[ID_Num].ki_output > PID_Control_Num[ID_Num].integral_limit) {
        PID_Control_Num[ID_Num].ki_output = PID_Control_Num[ID_Num].integral_limit;
    }else if (PID_Control_Num[ID_Num].ki_output < -PID_Control_Num[ID_Num].integral_limit){
        PID_Control_Num[ID_Num].ki_output = -PID_Control_Num[ID_Num].integral_limit;
    }
    PID_Control_Num[ID_Num].kp_output = PID_Control_Num[ID_Num].kp * PID_Control_Num[ID_Num].now_err;
    PID_Control_Num[ID_Num].kd_output = PID_Control_Num[ID_Num].kd * (PID_Control_Num[ID_Num].now_err - PID_Control_Num[ID_Num].last_err);
    PID_Control_Num[ID_Num].output = PID_Control_Num[ID_Num].ki_output + PID_Control_Num[ID_Num].kp_output + PID_Control_Num[ID_Num].kd_output;
    if (PID_Control_Num[ID_Num].output > PID_Control_Num[ID_Num].max_output){
        PID_Control_Num[ID_Num].output = PID_Control_Num[ID_Num].max_output;
    }else if (PID_Control_Num[ID_Num].output < -PID_Control_Num[ID_Num].max_output){
        PID_Control_Num[ID_Num].output = -PID_Control_Num[ID_Num].max_output;
    }
    return PID_Control_Num[ID_Num].output;
}

void Base_Speed_Calculation(uint8_t ID_Num, float Set_Speed)
{
    PID_Control_Num[ID_Num].last_err = PID_Control_Num[ID_Num].now_err;
    PID_Control_Num[ID_Num].now_err = Set_Speed - Now_Speed_Normal[ID_Num];
    if (abs(PID_Control_Num[ID_Num].now_err) < PID_Control_Num[ID_Num].deadband)
    {
      Current[ID_Num] = PID_Control_Num[ID_Num].output;
    }else{
      Current[ID_Num] = PID_Calculate(Set_Speed,ID_Num);
    }
}

void Base_Control_Init()
{
    for (int i=1;i<=4;i++) M3508_Speed_Control_Init(i);
		CAN1_Filter_Init();
}
