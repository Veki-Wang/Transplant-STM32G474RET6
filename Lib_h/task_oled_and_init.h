#ifndef _TASK_OLED_AND_INIT_H
#define _TASK_OLED_AND_INIT_H

#include "stm32g4xx_hal.h"
#include <string.h>
#include <stdint.h>
#include "OLED.h"
#include "main.h"
  

typedef  struct
{
    char *open_init_oled;
    uint8_t size_oled;
    uint8_t size_num;
    uint8_t size_float[2];

    char* start_flag;
    char* question_num;
    char* scan;

    char* pid_x_inr;
    char* pid_x_out;  
    char* pid_y_out;

    char* yaw;
    char* init_yaw;

    uint8_t oled_transfer_flag;

}oled_config;

extern oled_config oled_init;



void task_oled_and_init(void);
void oled_task(void);
#endif // _TASK_H
