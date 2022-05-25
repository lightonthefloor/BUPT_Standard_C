//
// Created by Trisoil on 2022/3/30.
//

#include <math.h>
#include "Location_Module_Recieve.h"
#include "Usart_Printf.h"
#include "CAN_Operation.h"
#include "usart.h"
#define R 0.16f
#define PI 3.1415926f

static union
{
		uint8_t data[24];
		float ActVal[6];
}VegaData_u;

union
{
		float Position;
		uint8_t Input;
}transmit_data_u[2];

Location Location_Data;
uint8_t Locator_Rx_Data[2][99];
int start_locator = 0;
int x,y,a;

extern  DMA_HandleTypeDef hdma_usart1_rx;

void Locator_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num) {
	//enable the DMA transfer for the receiver request
	//使能DMA串口接收
	SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);

	//enalbe idle interrupt
	//使能空闲中断
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	//HAL_UART_Receive_IT()
	//disable DMA
	//失效DMA
	__HAL_DMA_DISABLE(&hdma_usart1_rx);
	while (hdma_usart1_rx.Instance->CR & DMA_SxCR_EN) {
		__HAL_DMA_DISABLE(&hdma_usart1_rx);
	}

	hdma_usart1_rx.Instance->PAR = (uint32_t) &(USART1->DR);
	//memory buffer 1
	//内存缓冲区1
	hdma_usart1_rx.Instance->M0AR = (uint32_t) (rx1_buf);
	//memory buffer 2
	//内存缓冲区2
	hdma_usart1_rx.Instance->M1AR = (uint32_t) (rx2_buf);
	//data length
	//数据长度
	hdma_usart1_rx.Instance->NDTR = dma_buf_num;
	//enable double memory buffer
	//使能双缓冲区
	SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);

	//enable DMA
	//使能DMA
	__HAL_DMA_ENABLE(&hdma_usart1_rx);

}

void Locator_Rx_Init() {
	Locator_init(Locator_Rx_Data[0], Locator_Rx_Data[1], SBUS_RX_BUF_NUM);
}

void Locator_Data_Dealer(const uint8_t *sbus_buf)
{
	for (int i=0;i<99;i++)
	{
		if ((int) sbus_buf[i] == 0x0d &&
				(int) sbus_buf[i + 1] == 0x0a &&
				i < 73 && (int) sbus_buf[i + 26] == 0x0a &&
				(int) sbus_buf[i + 27] == 0x0d){
			for (int j=0;j<24;j++){
				VegaData_u.data[j] = sbus_buf[i + j + 2];
			}
		}
	}
	start_locator = 1;
	Location_Data.Pos_X = VegaData_u.ActVal[3] / 1000.0f;
	Location_Data.Angle = VegaData_u.ActVal[0];
	Location_Data.Pos_Y = VegaData_u.ActVal[4] / 1000.0f;
	float Angle = PI*(Location_Data.Angle/180.0f);
	Location_Data.Pos_X += R*((float)sin((double)Angle));
	Location_Data.Pos_Y += R;
	Location_Data.Pos_Y -= R*((float)cos((double)Angle));
	//Usart_Printf("X:%d    Y:%d\r\n",Location_Data.Pos_X,Location_Data.Pos_Y);
	x = (int)(Location_Data.Pos_X * 1000.0f);
	y = (int)(Location_Data.Pos_Y * 1000.0f);
	a = (int)(Location_Data.Angle * 100.0f);
	CAN_Transmit_Message('A',x,y,a,0,0x301);
}

void USART1_IRQHandler(void) {
	if (huart1.Instance->SR & UART_FLAG_RXNE)//接收到数据
	{
		__HAL_UART_CLEAR_PEFLAG(&huart1);
	} else if (USART1->SR & UART_FLAG_IDLE) {
		static uint16_t this_time_rx_len = 0;

		__HAL_UART_CLEAR_PEFLAG(&huart1);

		if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET) {
			/* Current memory buffer used is Memory 0 */

			//disable DMA
			//失效DMA
			__HAL_DMA_DISABLE(&hdma_usart1_rx);

			//get receive data length, length = set_data_length - remain_length
			//获取接收数据长度,长度 = 设定长度 - 剩余长度
			this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

			//reset set_data_lenght
			//重新设定数据长度
			hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

			//set memory buffer 1
			//设定缓冲区1
			hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;

			//enable DMA
			//使能DMA
			__HAL_DMA_ENABLE(&hdma_usart1_rx);

			if (this_time_rx_len){
				Locator_Data_Dealer(Locator_Rx_Data[0]);
			}
		} else {
			/* Current memory buffer used is Memory 1 */
			//disable DMA
			//失效DMA
			__HAL_DMA_DISABLE(&hdma_usart1_rx);

			//get receive data length, length = set_data_length - remain_length
			//获取接收数据长度,长度 = 设定长度 - 剩余长度
			this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

			//reset set_data_length
			//重新设定数据长度
			hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

			//set memory buffer 0
			//设定缓冲区0
			DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

			//enable DMA
			//使能DMA
			__HAL_DMA_ENABLE(&hdma_usart1_rx);

			if (this_time_rx_len){
				Locator_Data_Dealer(Locator_Rx_Data[1]);
			}
		}
	}
}