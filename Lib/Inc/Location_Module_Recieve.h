//
// Created by Trisoil on 2022/3/30.
//
#include "main.h"

#ifndef LOCATION_MODULE_C_LOCATION_MODULE_RECIEVE_H
#define LOCATION_MODULE_C_LOCATION_MODULE_RECIEVE_H

#define SBUS_RX_BUF_NUM 99u

typedef struct
{
		float Angle;
		float Pos_X;
		float Pos_Y;
}Location;

extern uint8_t Locator_Rx_Data[2][99];
extern float Pos_x;
extern int start_locator;
extern Location Location_Data;
extern int x,y,a;

extern void Locator_Rx_Init();

#endif //LOCATION_MODULE_C_LOCATION_MODULE_RECIEVE_H
