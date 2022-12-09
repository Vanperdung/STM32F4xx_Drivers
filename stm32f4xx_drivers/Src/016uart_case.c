/*
 * 016uart_case.c
 *
 *  Created on: Nov 10, 2021
 *      Author: dung
 */

#include "stm32f407xx.h"
#include <stdio.h>
#include <string.h>

char *msg[3] = {"Hello:3", "I am Dung 2001", "Nice to meet U!!!"};
char rx_buf[1024];

uint8_t rxCmplt = RESET;
uint8_t g_data = 0;
USART_Handle_t Usart2Handle;

extern void initialise_monitor_handles();

void delay(void)
{
	for(uint32_t i = 0; i < 500000; i++);
}

int main(void)
{
	uint32_t cnt = 0;

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
	UsartGpiox.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_3;
	GPIO_Init(&UsartGpiox);

	Usart2Handle.pUSARTx = USART2;
	Usart2Handle.USART_Config.USART_Mode = USART_MODE_TXRX;
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
		// Next message, make sure that cnt value doesn't cross 2
		cnt = cnt % 3;

		// Enable he receive interrupt
		while(USART_ReceiveDataIT(&Usart2Handle,(uint8_t*)rx_buf, strlen(msg[cnt])) != USART_READY);

		// Send the msg
		USART_SendData(&Usart2Handle, (uint8_t*)msg[cnt], strlen(msg[cnt]));

		printf("Transmitted: %s\n", msg[cnt]);

		// Wait until all the bytes are received from Arduino
		// When all the bytes are received, rxCmplt will be SET in application callback
		while(rxCmplt != SET);

		// Just make sure that last byte should be null otherwise %s fails while printing
		rx_buf[strlen(msg[cnt])+ 1] = '\0';

		// Print what we received from the Arduino
		printf("Received    : %s\n",rx_buf);

		// Invalidate the flag
		rxCmplt = RESET;

		// Move on to next message indexed in msg[]
		cnt ++;
	}
}

void USART2_IRQHandler(void)
{
	USART_IRQHandling(&Usart2Handle);
}


void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t ApEv)
{
   if(ApEv == USART_EVENT_RX_CMPLT)
   {
			rxCmplt = SET;

   }else if (ApEv == USART_EVENT_TX_CMPLT)
   {
	   ;
   }
}
