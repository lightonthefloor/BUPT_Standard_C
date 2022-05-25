//
// Created by Trisoil on 2022/2/23.
//
#include "usart.h"

#ifndef REMOTE_CONTROL_REMOTE_CONTROLLER_H
#define REMOTE_CONTROL_REMOTE_CONTROLLER_H

#define SBUS_RX_BUF_NUM 36u
#define RC_FRAME_LENGTH 18u
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)

typedef struct
{
    struct
    {
        int16_t ch[5];
        uint8_t s[2];
    } Remote_Controller;

    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t Press_L;
        uint8_t Press_R;
    }Mouse;

    struct
    {
        uint16_t v;
    }Keyboard;

}RC_Ctrl_s;

extern void Remote_Controller_Init();
extern void Data_Dealer(const uint8_t *sbus_buf, RC_Ctrl_s *rc_ctrl);
extern void Remote_Moving_Control();

extern RC_Ctrl_s Remote_Controller_Data;
extern uint8_t SBUS_RX_Buf[2][SBUS_RX_BUF_NUM];
extern int start_Remote;

#endif //REMOTE_CONTROL_REMOTE_CONTROLLER_H
