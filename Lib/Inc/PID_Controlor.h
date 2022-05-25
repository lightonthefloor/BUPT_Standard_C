//
// Created by Trisoil on 2021/12/14.
//
#include "gpio.h"

#ifndef UNTITLED_PID_CONTROLOR_H
#define UNTITLED_PID_CONTROLOR_H

typedef struct {
    float kp;
    float ki;
    float kd;
    float kp_output;
    float ki_output;
    float kd_output;
    float output;
    int err_sum;
    int now_err;
    int last_err;
    int deadband;
    int max_output;
    int integral_limit;
    int max_err;
}PID_Number;



extern PID_Number PID_Control_Num[8];
extern int16_t Current[5];

extern void M3508_Speed_Control_Init(uint8_t ID_Num);
extern void Base_Speed_Calculation(uint8_t ID_Num, float Set_Speed);
extern void Base_Control_Init();

#endif //UNTITLED_PID_CONTROLOR_H
