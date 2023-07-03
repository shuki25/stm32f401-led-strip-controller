/*
 * Filename: splash.c
 * Created Date: Friday, June 16th 2023, 3:21:10 pm
 * Author: Dr. Joshua Butler
 * 
 * Copyright (c) 2023 Joshua Butler
 */

#include <stdio.h>
#include <string.h>
#include "splash.h"
#include "cmsis_os.h"

void splash(led_strip_t *led)
{
    uint8_t i;
	uint8_t buffer[12];
	
    ssd1306_Init();
	ssd1306_Fill(Black);
	ssd1306_SetCursor(16, 3);
	ssd1306_WriteString("BUTLER", Font_16x26, White);
	ssd1306_UpdateScreen();
    led_strip_clear(led);
    led_strip_fill(led, 64, 0, 0);
    led_strip_set_brightness(led, 25);
    led_strip_WS2812_send(led);
    osDelay(1500);

    ssd1306_Fill(Black);
	ssd1306_SetCursor(3, 7);
	ssd1306_WriteString("ELECTRONICS", Font_11x18, White);
	ssd1306_UpdateScreen();
    led_strip_clear(led);
    led_strip_fill(led, 0, 64, 0);
    led_strip_set_brightness(led, 25);
    led_strip_WS2812_send(led);
    osDelay(1500);

    ssd1306_Fill(Black);
    ssd1306_SetCursor(20, 7);
    ssd1306_WriteString("(C) 2023", Font_11x18, White);
    ssd1306_UpdateScreen();
    led_strip_clear(led);
    led_strip_fill(led, 0, 0, 64);
    led_strip_set_brightness(led, 25);
    led_strip_WS2812_send(led);
    osDelay(1500);
    
    ssd1306_Fill(Black);
	snprintf((char *)buffer, 12, "v%d.%d", VERSION_MAJOR, VERSION_MINOR);
	uint8_t len = strlen((char *)buffer);
	ssd1306_SetCursor((128 - (len * 11)) >> 1, 7);
	ssd1306_WriteString((char *)buffer, Font_11x18, White);
	ssd1306_UpdateScreen();
    led_strip_clear(led);
    led_strip_set_brightness(led, 25);
    led_strip_WS2812_send(led);
    osDelay(1500);

    ssd1306_Fill(Black);
	osDelay(1500);
}
