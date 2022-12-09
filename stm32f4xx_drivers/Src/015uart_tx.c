/*
 * 015uart_tx.c
 *
 *  Created on: Nov 10, 2021
 *      Author: dung
 */

// Baudrate : 115200 bps
// Frame format: 1 stop bit, 8 data bits, no parity

#include "stm32f407xx.h"
#include<stdio.h>
#include<string.h>

char msg[1024] = "UART Tx testing ...\n\r";


void delay(void)
{
	for(uint32_t i = 0; i < 500000; i++);
}

int main(void)
{
	GPIO_Handle_t UsartGpiox;
	UsartGpiox.pGPIOx = GPIOA;
	UsartGpiox.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	UsartGpiox.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	UsartGpiox.GPIO_PinConfig.GPIO_PinAltFunMode = 7;

	//USART2_TX
	UsartGpiox.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_2;
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&UsartGpiox);

	//USART3_RX
//	UsartGpiox.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_3;
//	GPIO_Init(&UsartGpiox);

	USART_Handle_t Usart2Handle;
	Usart2Handle.pUSARTx = USART2;
	Usart2Handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	Usart2Handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	Usart2Handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	Usart2Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	Usart2Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	Usart2Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;

	USART_Init(&Usart2Handle);

	USART_PeripheralControl(USART2, ENABLE);
	USART_PeriClockControl(USART2, ENABLE);

	while(1)
	{
		delay();
		USART_SendData(&Usart2Handle, (uint8_t*)msg, strlen(msg));
	}
}

