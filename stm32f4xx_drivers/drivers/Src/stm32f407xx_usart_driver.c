/*
 * stm32f407xx_usart_driver.c
 *
 *  Created on: Nov 5, 2021
 *      Author: dung
 */


#include "stm32f407xx_usart_driver.h"

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pUSARTx->CR1 |= (1 << 13);
	}else
	{
		pUSARTx->CR1 &= ~(1 << 13);
	}
}

void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PLCK_EN();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PLCK_EN();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PLCK_EN();
		}
		else if(pUSARTx == UART4)
		{
			UART4_PLCK_EN();
		}
		else if(pUSARTx == UART5)
		{
			UART5_PLCK_EN();
		}
		else if(pUSARTx == USART6)
		{
			USART6_PLCK_EN();
		}
	}
	else
	{
		if(pUSARTx == USART1)
		{
			USART1_PLCK_DI();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PLCK_DI();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PLCK_DI();
		}
		else if(pUSARTx == UART4)
		{
			UART4_PLCK_DI();
		}
		else if(pUSARTx == UART5)
		{
			UART5_PLCK_DI();
		}
		else if(pUSARTx == USART6)
		{
			USART6_PLCK_DI();
		}
	}
}

void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t Baudrate)
{
	// Variable to hold the APB clock
	uint32_t PCLKx;

	// Variables to hold Mantissa and Fraction values
	uint32_t M_part, F_part;

	uint32_t usartdiv;

	uint32_t tempreg = 0;

	// Get the value of APB bus clock in to the variable PCLKx
	if(pUSARTx == USART1 || pUSARTx == USART6)
	{
		// USART1 and USART6 are hanging on APB2 bus
		PCLKx = RCC_GetPCLK2Value();
	}
	else
	{
		// USART2, USART3, UART4, UART5 are hanging on APB1 bus
		PCLKx = RCC_GetPCLK1Value();
	}

	// Check for OVER8 configuration bit
	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
		// OVER8 = 1, over sampling by 8
		usartdiv = ((25 * PCLKx) / (2 * Baudrate));
	}
	else
	{
		// OVER8 = 0, over sampling by 16
		usartdiv = ((25 * PCLKx) / (4 * Baudrate));
	}
	// Calculate the Mantissa part and the Fraction part
	M_part = usartdiv / 100;
	tempreg |= (M_part << 4);
	F_part = usartdiv - M_part * 100;
	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
	{
		// OVER8 = 1, oversampling by 8
		F_part = (((F_part * 8) + 50) / 100) & (uint8_t)0x0F;
	}
	else
	{
		// OVER8 = 0, oversampling by 16
		F_part = (((F_part * 16) + 50) / 100) & (uint8_t)0x0F;
	}
	//Place the fractional part in appropriate bit position . refer USART_BRR
	tempreg |= F_part;

	//copy the value of tempreg in to BRR register
	pUSARTx->BRR = tempreg;
}

void USART_Init(USART_Handle_t *pUSARTHandle)
{
	uint32_t tempreg1 = 0, tempreg2 = 0, tempreg3 = 0;
	// Implement the code to enable the clock for given USART peripheral
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		tempreg1 |= (1 << USART_CR1_TE);
	}
	else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		tempreg1 |= (1 << USART_CR1_RE);
	}
	else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		tempreg1 |= ((1 << USART_CR1_TE) | (0x1 << USART_CR1_RE));
	}

	// Configure M bit for the word length
	tempreg1 |= (pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M);

	//Configuration of parity control bit fields
	if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD)
	{
		//Implement the code to enable the parity control
		tempreg1 |= (1 << USART_CR1_PCE);
		//Implement the code to enable ODD parity
		tempreg1 |= (1 << USART_CR1_PS);
	}
	else if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		//Implement the code to enable the parity control
		tempreg1 |= (1 << USART_CR1_PCE);
		//Implement the code to enable ODD parity
		tempreg1 &= ~(1 << USART_CR1_PS);
	}
	else if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
	{
		//Implement the code to disable the parity control
		tempreg1 &= ~(1 << USART_CR1_PCE);
	}

	//Implement the code to configure the number of stop bits
	tempreg2 |= (pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP);

	//Configuration of USART hardware flow control
	if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		//Implement the code to enable CTS flow control
		tempreg3 |= ( 1 << USART_CR3_CTSE);


	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		//Implement the code to enable RTS flow control
		tempreg3 |= ( 1 << USART_CR3_RTSE);

	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		//Implement the code to enable both CTS and RTS Flow control
		tempreg3 |= ( 1 << USART_CR3_CTSE);
		tempreg3 |= ( 1 << USART_CR3_RTSE);
	}

	//Program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = tempreg1;
	//Program the CR2 register
	pUSARTHandle->pUSARTx->CR2 = tempreg2;
	//Program the CR3 register
	pUSARTHandle->pUSARTx->CR3 = tempreg3;

	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);
}

void DeInit(USART_RegDef_t *pUSARTx)
{
	if(pUSARTx == USART1)
	{
		USART1_REG_RESET();
	}
	else if(pUSARTx == USART2)
	{
		USART2_REG_RESET();
	}
	else if(pUSARTx == USART3)
	{
		USART3_REG_RESET();
	}
	else if(pUSARTx == UART4)
	{
		UART4_REG_RESET();
	}
	else if(pUSARTx == UART5)
	{
		UART5_REG_RESET();
	}
	else if(pUSARTx == USART6)
	{
		USART6_REG_RESET();
	}
}

void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint16_t *pdata;

	//Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0; i < Len; i++)
	{
		// Wait until TXE flag is set in the SR
		while(!(pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TXE)));

		// Check the word length (9 bits or 8 bits) in a frame
		if(pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_M))
		{
			// 9 bits in a data frame
			pdata = (uint16_t*)pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			// check for parity control
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				// No parity is used in this transfer
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				// The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			// 8 bits in a data frame
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer & (uint8_t)0xFF);
			pTxBuffer++;
		}
		// Wait until TC flag is set in the SR register
		while(!(pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TC)));
	}
}

void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	// Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0; i < Len; i++)
	{
		// Wait until RXNE flag is set in the SR register
		while(!(pUSARTHandle->pUSARTx->SR & (1 << USART_SR_RXNE)));

		// Check the word length to decide whether we are going to receive 9 bits or 8 bits
		if(pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_M))
		{
			// 9 bit in a data

			// Check using parity or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				// No parity is used. 9 bits will be user data
				*((uint16_t*)pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t)0x01FF);
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				// Parity is used. 8 bits data and 1 bit parity
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
				pRxBuffer++;
			}
		}
		else
		{
			// We are going to receive 8bit data in a frame

			// Check using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
				 *pRxBuffer =(uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
				 pRxBuffer++;
			}
			else
			{

				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//read only 7 bits, mask the DR with 0X7F
				 *pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);
				 pRxBuffer++;
			}
		}
	}
}

uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t txstate = pUSARTHandle->TxBusyState;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxBusyState = USART_BUSY_IN_TX;

		// Enable interrupt for TXE
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);

		// Enable interrupt for TC
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);
	}

	return txstate;
}

uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rxstate = pUSARTHandle->RxBusyState;

	if(rxstate != USART_BUSY_IN_RX)
	{
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxBusyState = USART_BUSY_IN_RX;

		(void)pUSARTHandle->pUSARTx->DR;

		//Implement the code to enable interrupt for RXNE
		pUSARTHandle->pUSARTx->CR1 |= ( 1 << USART_CR1_RXNEIE);
	}

	return rxstate;
}


void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber < 32)
		{
			//program ISER0 (interrupt set-enable register)
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ISER1 (interrupt set-enable register)
			*NVIC_ISER1 |= (1 << IRQNumber % 32);
		}
		else if(IRQNumber > 63 && (IRQNumber < 96))
		{
			//program ISER2 (interrupt set-enable register)
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	if(EnorDi == DISABLE)
	{
		if(IRQNumber < 32)
		{
			//program ICER0 (interrupt clear-enable register)
			*NVIC_ICER0 |= (1 << IRQNumber);

		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ICER1 (interrupt clear-enable register)
			*NVIC_ICER1 |= (1 << IRQNumber % 32);
		}
		else if(IRQNumber > 63 && IRQNumber < 96)
		{
			//program ICER2 (interrupt clear-enable register)
			*NVIC_ICER2 |= (1 << IRQNumber % 64);
		}
	}
}

// IRQ Priority
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. Find out the IPR register
	uint8_t IPRx = IRQNumber / 4;
	uint8_t IPRx_Section = IRQNumber % 4;

	uint8_t shift_amount = (8 * IPRx_Section) + 4;
	*(NVIC_PR_BASEADDR + IPRx) |= (IRQPriority << shift_amount);
}

void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{
	uint32_t temp1, temp2;

	uint16_t *pdata;

	/**********************************************Check for the TC flag********************************************************/

	// Check the state of TC bit in SR register
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TC);
	// Check the state of TCIE bit in CR1 register
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TCIE);

	if(temp1 && temp2)
	{
		// This is interrupt because of TC
		// Close transmission and call application callback if TxLen is zero

		if ( pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			// Check the TxLen. If it is zero then close the data transmission
			if(!(pUSARTHandle->TxLen))
			{
				//Implement the code to clear the TC flag
				pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_TC);

				//Implement the code to clear the TCIE control bit
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TCIE);

				//Reset the application state
				pUSARTHandle->TxBusyState = USART_READY;

				//Reset Buffer address to NULL
				pUSARTHandle->pTxBuffer = NULL;

				//Reset the length to zero
				pUSARTHandle->TxLen = 0;

				//Call the application call back with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_TX_CMPLT);
			}
		}
	}
	/*************************Check for TXE flag ********************************************/

	//Implement the code to check the state of TXE bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_TXE);

	//Implement the code to check the state of TXEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_TXEIE);


	if(temp1 && temp2)
	{
		//this interrupt is because of TXE
		if(pUSARTHandle->TxBusyState == USART_BUSY_IN_TX)
		{
			//Keep sending data until Txlen reaches to zero
			if(pUSARTHandle->TxLen > 0)
			{
				//Check the USART_WordLength item for 9BIT or 8BIT in a frame
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//if 9BIT load the DR with 2bytes masking  the bits other than first 9 bits
					pdata = (uint16_t*) pUSARTHandle->pTxBuffer;
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

					//check for USART_ParityControl
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used in this transfer , so 9bits of user data will be sent
						//Implement the code to increment pTxBuffer twice
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen -= 2;
					}
					else
					{
						//Parity bit is used in this transfer . so 8bits of user data will be sent
						//The 9th bit will be replaced by parity bit by the hardware
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen -= 1;
					}
				}
				else
				{
					//This is 8bit data transfer
					pUSARTHandle->pUSARTx->DR = (*pUSARTHandle->pTxBuffer  & (uint8_t)0xFF);

					//Implement the code to increment the buffer address
					pUSARTHandle->pTxBuffer++;
					pUSARTHandle->TxLen -= 1;
				}

			}
			if (pUSARTHandle->TxLen == 0)
			{
				//TxLen is zero
				//Implement the code to clear the TXEIE bit (disable interrupt for TXE flag )
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE);
			}
		}
	}

	/*************************Check for RXNE flag ********************************************/

	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_RXNE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE);


	if(temp1 && temp2)
	{
		//this interrupt is because of RXNE
		if(pUSARTHandle->RxBusyState == USART_BUSY_IN_RX)
		{
			if(pUSARTHandle->RxLen > 0)
			{
				//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//We are going to receive 9bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used , so all 9bits will be of user data

						//read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t*) pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t)0x01FF);

						//Now increment the pRxBuffer two times
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->RxLen -= 2;
					}
					else
					{
						//Parity is used, so 8bits will be of user data and 1 bit is parity
						 *pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
						 pUSARTHandle->pRxBuffer++;
						 pUSARTHandle->RxLen -= 1;
					}
				}
				else
				{
					//We are going to receive 8bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used , so all 8bits will be of user data

						//read 8 bits from DR
						 *pUSARTHandle->pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

					}

					else
					{
						//Parity is used, so , 7 bits will be of user data and 1 bit is parity

						//read only 7 bits , hence mask the DR with 0X7F
						 *pUSARTHandle->pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);

					}

					//Now , increment the pRxBuffer
					pUSARTHandle->pRxBuffer++;
					pUSARTHandle->RxLen -= 1;
				}


			}
			//if of >0

			if(!(pUSARTHandle->RxLen))
			{
				//disable the RXNE
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_RXNEIE);
				pUSARTHandle->RxBusyState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_RX_CMPLT);
			}
		}
	}


	/*************************Check for CTS flag ********************************************/
	//Note : CTS feature is not applicable for UART4 and UART5

	//Implement the code to check the status of CTS bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & (1 << USART_SR_CTS);

	//Implement the code to check the state of CTSE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_CTSE);

	if(temp1 && temp2)
	{
		//Implement the code to clear the CTS flag in SR
		pUSARTHandle->pUSARTx->SR &=  ~(1 << USART_SR_CTS);

		//this interrupt is because of CTS
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_CTS);
	}

	/*************************Check for IDLE detection flag ********************************************/

	//Implement the code to check the status of IDLE flag bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_IDLE);

	//Implement the code to check the state of IDLEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_IDLEIE);


	if(temp1 && temp2)
	{
		//Implement the code to clear the IDLE flag. Refer to the RM to understand the clear sequence
		temp1 = pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_IDLE);

		//this interrupt is because of idle
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_IDLE);
	}

	/*************************Check for Overrun detection flag ********************************************/

	//Implement the code to check the status of ORE flag  in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & USART_SR_ORE;

	//Implement the code to check the status of RXNEIE  bit in the CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & USART_CR1_RXNEIE;


	if(temp1  && temp2)
	{
		//Need not to clear the ORE flag here, instead give an API for the application to clear the ORE flag .

		//this interrupt is because of Overrun error
		USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
	}



	/*************************Check for Error Flag ********************************************/

	//Noise Flag, Overrun error and Framing Error in multibuffer communication
	//We don't discuss multibuffer communication in this course. please refer to the RM
	//The blow code will get executed in only if multibuffer mode is used.

	temp2 =  pUSARTHandle->pUSARTx->CR3 & (1 << USART_CR3_EIE) ;

	if(temp2)
	{
		temp1 = pUSARTHandle->pUSARTx->SR;
		if(temp1 & (1 << USART_SR_FE))
		{
			/*
				This bit is set by hardware when a de-synchronization, excessive noise or a break character
				is detected. It is cleared by a software sequence (an read to the USART_SR register
				followed by a read to the USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_FE);
		}

		if(temp1 & (1 << USART_SR_NE))
		{
			/*
				This bit is set by hardware when noise is detected on a received frame. It is cleared by a
				software sequence (an read to the USART_SR register followed by a read to the
				USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_NE);
		}

		if(temp1 & (1 << USART_SR_ORE))
		{
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERR_ORE);
		}
	}
}

__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t event)
{

}





