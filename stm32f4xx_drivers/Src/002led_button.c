

#include "stm32f407xx.h"

void delay1(void)
{
	for(uint32_t i = 0; i < 10000; i++);
}

int main(void)
{
	GPIO_Handle_t GPIOLed, GPIOButton;
	GPIOLed.pGPIOx = GPIOD;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIOButton.pGPIOx = GPIOD;
	GPIOButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_11;
	GPIOButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GPIOButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GPIOLed);
	GPIO_Init(&GPIOButton);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_11) == 0)
		{
			delay1();
			if(GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_11) == 0)
			{
				GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);
				while(GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_11) == 0);
			}
		}

	}
	return 0;
}
