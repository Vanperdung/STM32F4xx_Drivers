
#include "ds1307.h"
#include <string.h>

I2C_Handle_t ds1307_I2CHandle;

static void ds1307_i2c_pin_config(void)
{
    GPIO_Handle_t i2c_sda, i2c_scl;

    memset(&i2c_sda, 0, sizeof(i2c_sda));
    memset(&i2c_scl, 0, sizeof(i2c_scl));

    i2c_sda.pGPIOx = DS1307_I2C_GPIO_PORT;
    i2c_sda.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_SDA_PIN;
    i2c_sda.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    i2c_sda.GPIO_PinConfig.GPIO_PinAltFunMode = AF4;
    i2c_sda.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    i2c_sda.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD;
    i2c_sda.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIO_Init(&i2c_sda);

    i2c_scl.pGPIOx = DS1307_I2C_GPIO_PORT;
    i2c_scl.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_SCL_PIN;
    i2c_scl.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    i2c_scl.GPIO_PinConfig.GPIO_PinAltFunMode = AF4;
    i2c_scl.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    i2c_scl.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD;
    i2c_scl.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIO_Init(&i2c_scl);
}

static void ds1307_i2c_config(void)
{
    ds1307_I2CHandle.pI2Cx = DS1307_I2C;
    ds1307_I2CHandle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
    ds1307_I2CHandle.I2C_Config.I2C_SCLSpeed = DS1307_I2C_SPEED;
    I2C_Init(&ds1307_I2CHandle);
}

static void ds1307_write(uint8_t vaule, uint8_t reg_addr)
{
    uint8_t tx[2];
    tx[0] = reg_addr;
    tx[1] = vaule;
    I2C_MasterSendData(&ds1307_I2CHandle, tx, 2, DS1307_SLAVE_ADDR, I2C_DISABLE_SR);
}

static uint8_t ds1307_read(uint8_t reg_addr)
{
    uint8_t rx;
    I2C_MasterSendData(&ds1307_I2CHandle, &reg_addr, 1, DS1307_SLAVE_ADDR, I2C_ENABLE_SR);
    I2C_MasterReceiveData(&ds1307_I2CHandle, &rx, 1, DS1307_SLAVE_ADDR, I2C_DISABLE_SR);
    return rx;
}

static uint8_t binary_to_bcd(uint8_t time)
{
    uint8_t unit = 0, teen = 0, bcd;
    if(time >= 10)
    {
        unit = time % 10;
        teen = time / 10;
        bcd = (uint8_t)((unit << 4) | teen);
    }
    else
    {
        bcd = time;
    }
    return bcd;
}

static uint8_t bcd_to_binary(uint8_t time)
{
    uint8_t binary;
    binary = ((uint8_t)(time >> 4) * 10 + (time & (uint8_t)0x0F));
    return binary;
}

uint8_t ds1307_init(void)
{
    uint8_t clock_state;

    // 1. Init the I2C pins
    ds1307_i2c_pin_config();   
    // 2. Initialize the I2C peripheral
    ds1307_i2c_config();
    // 3. Enable the I2C peripheral
    I2C_PeripheralControl(ds1307_I2CHandle.pI2Cx, ENABLE);
    ds1307_I2CHandle.pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
    // 4. Make clock halt = 0
    ds1307_write(0x00, DS1307_ADDR_SECOND);
    // 5. Read back clock halt bit
    clock_state = ds1307_read(DS1307_ADDR_SECOND);

    return ((clock_state >> 7) & 0x1);

}

// Function prototypes
void ds1307_set_current_time(RTC_time_t *rtc_time)
{
    uint8_t seconds, hours;
    seconds = binary_to_bcd(rtc_time->seconds);
    seconds &= ~(1 << 7);
    ds1307_write(seconds, DS1307_ADDR_SECOND);
    ds1307_write(binary_to_bcd(rtc_time->minutes), DS1307_ADDR_MINUTE);
    hours = binary_to_bcd(rtc_time->hours);
    if(rtc_time->time_format == TIME_FORMAT_24HRS)
    {
        hours &= ~(1 << 6);
    }
    else 
    {
        hours |= (1 << 6);
        hours = (rtc_time->time_format == TIME_FORMAT_12HRS_PM) ? (hours | (1 << 5)) : (hours & ~(1 << 5));
    }
    ds1307_write(hours, DS1307_ADDR_HOUR);
}

void ds1307_get_current_time(RTC_time_t *rtc_time)
{
    uint8_t seconds, hours;
    seconds = ds1307_read(DS1307_ADDR_SECOND);
    seconds &= ~(1 << 7);
    rtc_time->seconds = bcd_to_binary(seconds);
    rtc_time->minutes = bcd_to_binary(ds1307_read(DS1307_ADDR_MINUTE));
    hours = bcd_to_binary(ds1307_read(DS1307_ADDR_HOUR));
    if(hours & (1 << 6)) // 12-hour mode
    {
        if(hours & (1 << 5)) // PM
        {
            rtc_time->time_format = TIME_FORMAT_12HRS_PM;
            hours &= ~((1 << 6) | (1 << 5));
        }
        else
        {
            rtc_time->time_format = TIME_FORMAT_12HRS_AM;
            hours &= ~((1 << 6) | (1 << 5));
        }
    }
    else // 24-hour mode
    {
        rtc_time->time_format = TIME_FORMAT_24HRS;
    }
    rtc_time->hours = bcd_to_binary(hours);
}

void ds1307_set_current_date(RTC_date_t *rtc_date)
{
    ds1307_write(binary_to_bcd(rtc_date->day), DS1307_ADDR_DAY);
    ds1307_write(binary_to_bcd(rtc_date->date), DS1307_ADDR_DATE);
    ds1307_write(binary_to_bcd(rtc_date->month), DS1307_ADDR_MONTH);
    ds1307_write(binary_to_bcd(rtc_date->year), DS1307_ADDR_YEAR);
}

void ds1307_get_current_date(RTC_date_t *rtc_date)
{
    rtc_date->day = bcd_to_binary(ds1307_read(DS1307_ADDR_DAY));
    rtc_date->date = bcd_to_binary(ds1307_read(DS1307_ADDR_DATE));
    rtc_date->month = bcd_to_binary(ds1307_read(DS1307_ADDR_MONTH));
    rtc_date->year = bcd_to_binary(ds1307_read(DS1307_ADDR_YEAR));
}
