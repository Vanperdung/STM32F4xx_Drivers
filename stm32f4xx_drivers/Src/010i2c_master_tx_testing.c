#include <stdio.h>
#include <string.h>
#include <stm32f407xx.h>

#define SLAVE_ADDR 0x68

/* I2C1
PB8: I2C1_SCL
PB7: I2C1_SDA */

GPIO_Handle_t gpiob, button;
I2C_Handle_t i2c1;

uint8_t data[] = "hello world!";

void delay(void)
{
    for(uint32_t i = 0; i < 250000; i++);
}

void Button_GPIOInit(void)
{
    button.pGPIOx = GPIOB;
    
    button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
    button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
    button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
    GPIO_Init(&button);
}

void I2C1_GPIOBInit(void)
{
    gpiob.pGPIOx = GPIOB;

    gpiob.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    gpiob.GPIO_PinConfig.GPIO_PinAltFunMode = AF4;
    gpiob.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    gpiob.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
    gpiob.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    // SCL
    gpiob.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_8;
    GPIO_Init(&gpiob);
    // SDA
    gpiob.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
    GPIO_Init(&gpiob);
}

void I2C1_Init(void)
{
    i2c1.pI2Cx = I2C1;

    i2c1.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
    i2c1.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

    I2C_Init(&i2c1);
}

int main(void)
{
    I2C1_GPIOBInit();
    I2C1_Init();
    Button_GPIOInit();

    I2C_PeripheralControl(i2c1.pI2Cx, ENABLE);

    while(1)
    {
        if(!(button.pGPIOx->IDR & (1 << GPIO_PIN_5)))
        {
            delay();
            if(!(button.pGPIOx->IDR & (1 << GPIO_PIN_5)))
            {
                I2C_MasterSendData(&i2c1, data, strlen((char*)data), SLAVE_ADDR, I2C_DISABLE_SR);        
            }
            while(!(button.pGPIOx->IDR & (1 << GPIO_PIN_5)));
        }
        
    }
}

