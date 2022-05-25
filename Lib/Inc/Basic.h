//
// Created by Trisoil on 2022/5/24.
//

#include "gpio.h"
#include "main.h"

#ifndef BUPT_STANDARD_C_BASIC_H
#define BUPT_STANDARD_C_BASIC_H

#define KEY_STATE HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin)

extern void Basic_Init();
extern void LED_R_On();
extern void LED_R_Off();
extern void LED_B_On();
extern void LED_B_Off();
extern void LED_G_On();
extern void LED_G_Off();

#endif //BUPT_STANDARD_C_BASIC_H
