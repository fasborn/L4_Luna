/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SK6812_H__
#define __SK6812_H__

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE END Private defines */

#include <stdint.h>

//--------------------------------------------------

#define DELAY_LEN 64
#define LED_COUNT 297
#define ARRAY_LEN DELAY_LEN*2 + LED_COUNT*32

#define HIGH 64
#define LOW 36


void fill_buffer(uint32_t color, uint16_t lower, uint16_t upper);
float get_position(float l_position);
void fill_led(uint32_t color, uint16_t led);
void fill_buff_leds(uint32_t color, uint16_t lower, uint16_t upper);
void fill_with_background(uint32_t color, uint16_t lower, uint16_t upper, uint32_t background_color);
void clear_buffer(void);

/* USER CODE BEGIN Prototypes */

#endif /*__ GPIO_H__ */

/************************END OF FILE****/
