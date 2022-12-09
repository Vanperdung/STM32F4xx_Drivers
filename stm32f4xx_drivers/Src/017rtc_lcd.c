#include <stdio.h>
#include <stdint.h>
#include "ds1307.h"

RTC_time_t current_time;
RTC_date_t current_date;

char *get_day_of_week(uint8_t count);
char *date_to_string(RTC_date_t *current_date);
char *time_to_string(RTC_time_t *current_time);

void ds1307_init_current(void)
{
    current_date.day = SUNDAY;
    current_date.date = 9;
    current_date.month = 1;
    current_date.year = 22;

    current_time.seconds = 0;
    current_time.minutes = 52;
    current_time.hours = 10;
    current_time.time_format = TIME_FORMAT_12HRS_PM;

    ds1307_set_current_date(&current_date);
    ds1307_set_current_time(&current_time);
}

int main(void)
{
    char *format;
    printf("RTC test\n");

    if(ds1307_init() != 0)
    {
        printf("RTC init has failed\n");
        while(1);
    }
    ds1307_init_current();

    ds1307_get_current_date(&current_date);
    ds1307_get_current_time(&current_time);

    if(current_time.time_format != TIME_FORMAT_24HRS)
    {
        format = (current_time.time_format) ? "PM" : "AM";
        printf("Current time = %s %s\n", time_to_string(&current_time), format);
    }
    else
    {
        printf("Current time = %s\n", time_to_string(&current_time));
    }

    printf("Current date = %s <%s>\n", date_to_string(&current_date), get_day_of_week(current_date.day));

    return 0;
}

void number_to_string(uint8_t num, char *buf)
{
    if(num < 10)
    {
        buf[0] = '0';
        buf[1] = num + 48;
    }
    else if(num >= 10 && num < 99)
    {
        buf[0] = (num / 10) + 48;
        buf[1] = (num % 10) + 48;
    }
}

char *time_to_string(RTC_time_t *current_time)
{
    char buf[9];
    buf[2] = ':';
    buf[5] = ':';

    number_to_string(current_time->hours, buf);
    number_to_string(current_time->minutes, &buf[3]);
    number_to_string(current_time->seconds, &buf[6]);
    buf[8] = '\0';

    return buf;
}

char *date_to_string(RTC_date_t *current_date)
{
    char buf[9];
    buf[2] = '/';
    buf[5] = '/';
    
    number_to_string(current_date->date, buf);
    number_to_string(current_date->month, &buf[3]);
    number_to_string(current_date->year, &buf[6]);

    buf[8] = '\0';
    
    return buf;
}

char *get_day_of_week(uint8_t count)
{
    char *days[] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
    return days[count - 1];
}
