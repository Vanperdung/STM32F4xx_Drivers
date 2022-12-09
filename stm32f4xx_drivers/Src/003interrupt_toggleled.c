

#include "stm32f407xx.h"

void delay()
{
	for(uint32_t i = 0; i < (500000 / 2); i++); //delay 200ms
}

void EXTI9_5_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_5);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_0);
}

int main(void)
{
	// led: PA0
	// button: PD5
	GPIO_Handle_t Led, Button;

	Led.pGPIOx = GPIOA;
	Led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	Led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	Led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	Led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	Led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&Led);

	Button.pGPIOx = GPIOD;
	Button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	Button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	Button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
	Button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&Button);

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, 30);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);




	while(1)
	{

	}
	return 0;
}
