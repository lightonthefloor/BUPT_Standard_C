//
// Created by Trisoil on 2022/2/23.
//

#include "Remote_Controller.h"
#include "PID_Controlor.h"
#include "CAN_Operation.h"
#include "Move_Controlor.h"
#include "usart.h"
#include "main.h"

extern DMA_HandleTypeDef hdma_usart3_rx;

RC_Ctrl_s Remote_Controller_Data;
uint8_t SBUS_RX_Buf[2][SBUS_RX_BUF_NUM];
int start_Remote = 0;

void Data_Dealer(const uint8_t *sbus_buf, RC_Ctrl_s *rc_ctrl);

void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    //enable the DMA transfer for the receiver request
    //使能DMA串口接收
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    //HAL_UART_Receive_IT()
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }

    hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    hdma_usart3_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart3_rx);

}

void Remote_Controller_Init()
{
    RC_init(SBUS_RX_Buf[0],SBUS_RX_Buf[1],SBUS_RX_BUF_NUM);
}

void USART3_IRQHandler(void)
{
    if(huart3.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);
    }
    else if(USART3->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;

            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                Data_Dealer(SBUS_RX_Buf[0], &Remote_Controller_Data);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_length
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数据
                Data_Dealer(SBUS_RX_Buf[1], &Remote_Controller_Data);
            }
        }
    }
}

void Data_Dealer(const uint8_t *sbus_buf, RC_Ctrl_s *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->Remote_Controller.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->Remote_Controller.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->Remote_Controller.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &0x07ff;
    rc_ctrl->Remote_Controller.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->Remote_Controller.s[1] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl->Remote_Controller.s[0] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl->Mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->Mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->Mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->Mouse.Press_L = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->Mouse.Press_R = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->Keyboard.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->Remote_Controller.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //Left Up Wheel value

    rc_ctrl->Remote_Controller.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->Remote_Controller.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->Remote_Controller.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->Remote_Controller.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->Remote_Controller.ch[4] -= RC_CH_VALUE_OFFSET;
}

void Remote_Moving_Control()
{
  float Omega,X_Speed,Y_Speed;
  if (Remote_Controller_Data.Remote_Controller.s[1] == 1)
  {
    X_Speed = (5000.0f/660.f)*(float)Remote_Controller_Data.Remote_Controller.ch[2];
    Y_Speed = (5000.0f/660.f)*(float)Remote_Controller_Data.Remote_Controller.ch[3];
    Omega = (100.0f/660.0f)*(float)Remote_Controller_Data.Remote_Controller.ch[0];
  }else if (Remote_Controller_Data.Remote_Controller.s[1] == 3)
  {
    X_Speed = (3500.0f/660.f)*(float)Remote_Controller_Data.Remote_Controller.ch[2];
    Y_Speed = (3500.0f/660.f)*(float)Remote_Controller_Data.Remote_Controller.ch[3];
    Omega = (100.0f/660.0f)*(float)Remote_Controller_Data.Remote_Controller.ch[0];
  }else if (Remote_Controller_Data.Remote_Controller.s[1] == 2)
  {
    X_Speed = (2000.0f/660.f)*(float)Remote_Controller_Data.Remote_Controller.ch[2];
    Y_Speed = (2000.0f/660.f)*(float)Remote_Controller_Data.Remote_Controller.ch[3];
    Omega = (100.0f/660.0f)*(float)Remote_Controller_Data.Remote_Controller.ch[0];
  }
  if (X_Speed == 0 && Y_Speed == 0 && Omega == 0)
  {
      for (int i=1;i<=4;i++)
      {
        PID_Control_Num[i].ki_output = 0;
        PID_Control_Num[i].kd_output = 0;
        PID_Control_Num[i].kp_output = 0;
        PID_Control_Num[i].output = 0;
      }
  }
  MecanumChassis_OmniDrive((float)-X_Speed,(float)Y_Speed,-Omega);
}
