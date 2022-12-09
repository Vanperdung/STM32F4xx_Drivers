#include "stm32f407xx_i2c_driver.h"

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummyRead;
	// Check device mode
	if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	{
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize == 1)
			{
				// Disable the Ack
				pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
				// Clear the ADDR flag
				dummyRead = pI2CHandle->pI2Cx->SR1;
				dummyRead = pI2CHandle->pI2Cx->SR2;
				(void)dummyRead;
			}
		}
		else
		{
			dummyRead = pI2CHandle->pI2Cx->SR1;
			dummyRead = pI2CHandle->pI2Cx->SR2;
			(void)dummyRead;
		}
	}
	else
	{
		dummyRead = pI2CHandle->pI2Cx->SR1;
		dummyRead = pI2CHandle->pI2Cx->SR2;
		(void)dummyRead;
	}
} 

static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr, uint8_t State)
{
	SlaveAddr = SlaveAddr << 1;
	if(State == WRITE)
	{
		// Send data: Write(0)
		SlaveAddr &= ~(1);
		pI2Cx->DR = SlaveAddr;
	}
	else
	{
		// Receive data: Read(1)
		SlaveAddr |= (1);
		pI2Cx->DR = SlaveAddr;
	}
}

static void I2C_ClearSTOPFFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummyRead;
	dummyRead = pI2CHandle->pI2Cx->SR1;
	pI2CHandle->pI2Cx->CR1 |= 0x0000;
	(void)dummyRead;
}

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		//TODO
	}
}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        pI2Cx->CR1 |= (1 << I2C_CR1_PE);
    }
    else
    {
        pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
    }
}

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
    uint32_t tempreg = 0;
	uint16_t ccr_value = 0;

	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	tempreg |= (pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK);
	pI2CHandle->pI2Cx->CR1 |= tempreg;

	tempreg = 0;
	tempreg |= (RCC_GetPCLK1Value() / 1000000U);
	pI2CHandle->pI2Cx->CR2 |= (tempreg & 0x3F);

	tempreg = 0;
	tempreg |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << I2C_OAR1_ADD71);
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 |= tempreg;

	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// Standard mode
		ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= (ccr_value & 0xFFF);
	}
	else
	{
		// Fast mode
		tempreg |= (1 << I2C_CCR_FS);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		else
		{
			tempreg |= (1 << I2C_CCR_DUTY);
			ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR |= tempreg;

	// TRISE Configuration
	// TRISE = f * t(rise) + 1 
	// t(rise Sm max) = 1000ns, t(rise Fm max) = 300ns
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// Standard mode
		tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1 ;

	}else
	{
		// Fast mode
		tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
}

void I2C_DeInit(I2C_Handle_t *pI2CHandle)
{

}

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	// 1. Generate the START condition
	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);
	// 2. Confirm that start generation is completed by checking the SB flag in the SR1
	while(!(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB)));
	// 3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr, WRITE);
	// 4. Confirm that address phase is completed by checking the ADDR flag in the SR1
	while(!(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR)));
	// 5. Clear the ADDR flag according to its software sequence
	// Note: Until ADDR is cleared SCL will be stretched (pull to LOW)
	I2C_ClearADDRFlag(pI2CHandle);
	// 6. Send the data until Len becomes 0
	while(Len > 0)
	{
		while(!(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE)));
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer++;
		Len--;
	}
	// 7. When Len becomes 0 wait for TXE = 1 and BTF = 1 before generating STOP condition 
	// Note: TXE = 1, BTF = 1 means that bit SR and DR are empty and next transmission should begin
	// When BTF = 1 SCL will be stretched (pull to LOW)
	while(!(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE)));
	while(!(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF)));
	// 8. Generate STOP condition and Masster need not to wait for the completion of STOP condition
	// Note: Generating STOP condition autimatically clears thr BTF
	if(Sr == I2C_DISABLE_SR)
	{
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
	}
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	// 1. Generate he START condition
	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);
	// 2. Confirm that START condituon generation is completed by checking the SB flag in the SR1
	// Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(!(pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_SB));
	// 3. Send the address of the slave with r/nw bit set to Read(1) (total 8 bits)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr, READ);
	// 4. Wait until address phase is completed by checking the ADDR flag in the SR1
	while(!(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR)));
	// Procedure to read only 1 byte from Slave
	if(Len == 1)
	{
		// Disable Acking 
		pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
		// Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);
		// Wait until RXNE becomes 1
		while(!(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE)));
		// Generate STOP conditon
		if(Sr == I2C_DISABLE_SR)
		{
			pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
		}
		// Read data in DR register
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}
	// Procedure to read data from Slave when Len > 1
	if(Len > 1)
	{
		// Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);
		
		// Read data in DR register until Len = 0
		for(uint32_t i = Len; i > 0; i--)
		{
			// Wait until RXNE becomes 1
			while(!(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE)));
			if(i == 2)
			{
				// Disable Acking
				pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
				// Generate STOP condition
				if(Sr == I2C_DISABLE_SR)
				{
					pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
				}
			}
			// Read data in DR register
			*pRxBuffer = pI2CHandle->pI2Cx->DR;
			pRxBuffer++;
		}
	}
	// Re_enable Acking
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}
}

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data)
{
	pI2Cx->DR = (uint32_t)data;
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
	return (uint8_t)pI2Cx->DR;
}

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxbuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;
		busystate = pI2CHandle->TxRxState;

		// Implement the code to generate START condition
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);
		// Implement the code to enable ITBUFEN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
		// Implement the code to enable ITEVFEN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVFEN);
		// Implement the code to enable ITERREN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}
	return busystate;
}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busystate = pI2CHandle->TxRxState;
	
	if((busystate != I2C_BUSY_IN_RX) && (busystate != I2C_BUSY_IN_TX))
	{
		pI2CHandle->pRxbuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;
		pI2CHandle->RxSize = Len; 
 		busystate = pI2CHandle->TxRxState;
		// Implement the code to generate START condition
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);
		// Implement the code to enable ITBUFEN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
		// Implement the code to enable ITEVFEN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVFEN);
		// Implement the code to enable ITERREN control bit
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}
	return busystate;
}

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);

		}
		else if(IRQNumber > 31 && IRQNumber < 64) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ISER2 register 
			*NVIC_ISER3 |= (1 << (IRQNumber % 64));
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 6 && IRQNumber < 96)
		{
			//program ICER2 register
			*NVIC_ICER3 |= (1 << (IRQNumber % 64));
		}
	}
}

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// Find out the IPR register
	uint8_t IPRx = IRQNumber / 4;
	uint8_t IPRx_Section = IRQNumber % 4;

	uint8_t shift_amount = (8 * IPRx_Section) + 4;
	*(NVIC_PR_BASEADDR + IPRx) |= (IRQPriority << shift_amount);
}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->TxLen > 0)
	{
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxbuffer);
		pI2CHandle->pTxbuffer++;
		pI2CHandle->TxLen--;
	}
}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->RxSize == 1)
	{
		*(pI2CHandle->pRxbuffer) = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
	}
	if(pI2CHandle->RxSize > 1)
	{
		if(pI2CHandle->RxLen == 2)
		{
			pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
		}
		*(pI2CHandle->pRxbuffer) = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxbuffer++;
		pI2CHandle->RxLen--;
	}
	if(pI2CHandle->RxLen == 0)
	{
		// Close the I2C data reception and notify the application

		// 1. Generate the STOP condition
		if(pI2CHandle->Sr == I2C_DISABLE_SR)
		{
			pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
		}
		// 2. Close the I2C RX
		I2C_CloseReceiveData(pI2CHandle);
		// 3. Notify the application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}

// I2C1_EV_IRQ_LINE: IRQn 31
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	// Interrupt handling for both Master and Slave mode of a device
	uint8_t temp1, temp2, temp3;

	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVFEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);
	// 1. Handle for interrupt generated by SB event
	// Note: SB flag is only applicable in Master mode
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);
	if(temp1 && temp3)
	{
		// The interrupt is generated because of SB event
		// This block won't be executed in Slave mode because SB bit is always 0 in Slave mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr, WRITE);
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, pI2CHandle->DevAddr, READ);
		}
	}
	// 2. Handle for interrupt generated by ADDR event (clear ADDR flag)
	// Note: When Master mode: Address is sent
	// 		 When Slave mode: Address matched with own address
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	if(temp1 && temp3)
	{
		I2C_ClearADDRFlag(pI2CHandle);
	}
	// 3. Handle for interrupt generated by STOPF event (stop received)
	// Note: STOP detection flag is applicable only Slave mode. 
	// 		 In Master mode this flag will never be set
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	if(temp1 && temp3)
	{
		// Clear the STOPF flag
		I2C_ClearSTOPFFlag(pI2CHandle);
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}
	// 4. Handle for interrupt generated by BTF event (data byte tranfer finished)
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	if(temp1 && temp3)
	{
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			// Make sure that TxE is also set and Len = 0
			if((pI2CHandle->TxLen == 0) && (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE)))
			{
				// 1. Generate the STOP condition
				if(pI2CHandle->Sr == I2C_DISABLE_SR)
				{
					pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
				}
				// 2. Reset all the member elements of the handle structure
				I2C_CloseSendData(pI2CHandle);
				// 3. Notice the application about transmission complete
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
			}
			
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			// Not do anything
		}
	}
	// 5. Handle for interrupt generated by RxNE event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
	if(temp1 && temp2 && temp3)
	{
		// Check device mode
		if(pI2CHandle->pI2Cx->SR2 & ( 1 << I2C_SR2_MSL))
		{
			// Master mode
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}
		else
		{
			// Slave receiver mode 
			if(!(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	} 
	// 6. Handle for interrupt generated by TxE event
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
	if(temp1 && temp2 && temp3)
	{
		// Check device mode
		if(!(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)))
		{
			// Master mode
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}
		else
		{
			// Slave transmitter mode 
			if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}		
		}
	}
}

// I2C1_ER_IRQ_LENE: IRQn 32
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1, temp2;

    // Know the status of ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & (1 << I2C_CR2_ITERREN);


	/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_BERR);
	if(temp1 && temp2)
	{
		// This is Bus error
		// Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);
		// Implement the code to notify the application about the error
	   	I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
	}

	/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_ARLO );
	if(temp1 && temp2)
	{
		// This is arbitration lost error
		// Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);
		// Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
	}

	/***********************Check for ACK failure  error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_AF);
	if(temp1 && temp2)
	{
		// This is ACK failure error
	    // Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);
		// Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
	}

	/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		// This is Overrun/underrun
	    // Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);
		// Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}

	/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_TIMEOUT);
	if(temp1 && temp2)
	{
		// This is Time out error
	    // Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);
		// Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}

}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVFEN);


	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxbuffer = NULL;
	pI2CHandle->TxLen = 0;
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVFEN);

	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxbuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}
}

__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv)
{

}
