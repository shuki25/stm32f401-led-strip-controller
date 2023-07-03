/*
* led_strip.c
* Created on: 2023-07-02
* Author: Joshua Butler, MD, MHI
*
* This file contains the functions for controlling the LED strip.
* The LED strip is controlled using the WS2812B LED chipset.
*
*/

#include "led_strip.h"
#include "misc.h"
#include <math.h>
#include <stdlib.h>

#define NUM_LEDS 128
#define USE_BRIGHTNESS 1
#define NUM_SACRIFICIAL_LED 1
#define PI 3.14159265358979323846

const uint8_t color_groups[7][3] = {
    { 255, 0, 0 },
    // red
    { 0, 255, 0 },
    // green
    { 0, 0, 255 }, // blue
    { 255, 255, 0 },
    // yellow
    { 255, 0, 255 },
    // magenta
    { 0, 255, 255 },
    // cyan
    { 255, 255, 255 } // white
};

/*
 * @brief Initializes the LED strip object.
 * 
 * @param led_obj Pointer to the LED strip object.
 * @param htim Pointer to the timer handle.
 * @param channel Timer channel.
 * @param counter_period Timer counter period.
 * @param num_leds Number of LEDs in the strip.
 * @param sacrificial_led_flag Flag to indicate whether to use a sacrificial LED.
 * 
 * @return led_strip_error_t Returns LED_STRIP_OK if successful, LED_STRIP_ERROR if not.
 **/

led_strip_error_t led_strip_init(led_strip_t *led_obj, TIM_HandleTypeDef *htim, const uint32_t channel, const uint8_t counter_period, const uint8_t num_leds, const uint8_t sacrificial_led_flag) {
    
    uint8_t offset;
    if (sacrificial_led_flag > 1)
    {
        return LED_STRIP_ERROR;
    }
    else
    {
        led_obj->sacrificial_led_flag = sacrificial_led_flag;
        offset = sacrificial_led_flag ? 1 : 0;
    }
    led_obj->htim = htim;
    led_obj->channel = channel;
    led_obj->counter_period = counter_period;
    led_obj->duty_high = round(counter_period * 2 / 3);
    led_obj->duty_low = round(counter_period * 1 / 3);
    led_obj->max_brightness = 45;
    led_obj->primary = NULL;
    led_obj->secondary = NULL;
    led_obj->working = NULL;
 
    if (!num_leds)
    {
        return LED_STRIP_ERROR;
    }
    led_obj->num_leds = num_leds > NUM_LEDS ? NUM_LEDS : num_leds;
    led_obj->data_sent_flag = 0;
    led_obj->primary = (uint8_t **)malloc((num_leds + offset) * sizeof(uint8_t *));
    led_obj->secondary = (uint8_t **)malloc((num_leds + offset) * sizeof(uint8_t *));
    led_obj->working = (uint8_t **)malloc((num_leds + offset) * sizeof(uint8_t *));
    if (led_obj->primary == NULL || led_obj->secondary == NULL || led_obj->working == NULL) {
        return LED_STRIP_MALLOC_FAILED;
    }
    for (int i = 0; i < num_leds + offset; i++) {
        led_obj->primary[i] = (uint8_t *)malloc(4 * sizeof(uint8_t));
        led_obj->secondary[i] = (uint8_t *)malloc(4 * sizeof(uint8_t));
        led_obj->working[i] = (uint8_t *)malloc(4 * sizeof(uint8_t));
        if (led_obj->primary[i] == NULL || led_obj->secondary[i] == NULL || led_obj->working[i] == NULL) {
            return LED_STRIP_MALLOC_FAILED;
        }
    }
    for (int i = 0; i < (num_leds + offset); i++) {
        led_obj->primary[i][0] = i;
        led_obj->primary[i][1] = 0;
        led_obj->primary[i][2] = 0;
        led_obj->primary[i][3] = 0;
        led_obj->secondary[i][0] = i;
        led_obj->secondary[i][1] = 0;
        led_obj->secondary[i][2] = 0;
        led_obj->secondary[i][3] = 0;
        led_obj->working[i][0] = i;
        led_obj->working[i][1] = 0;
        led_obj->working[i][2] = 0;
        led_obj->working[i][3] = 0;
    }
    led_obj->mod = (uint8_t **)malloc((num_leds + offset) * sizeof(uint8_t *));
    if (led_obj->mod == NULL) {
        return LED_STRIP_MALLOC_FAILED;
    }
    for (int i = 0; i < (num_leds + offset); i++) {
        led_obj->mod[i] = (uint8_t *)malloc(4 * sizeof(uint8_t));
        if (led_obj->mod[i] == NULL) {
            return LED_STRIP_MALLOC_FAILED;
        }
    }
    led_obj->pwm_data = (uint16_t *)malloc(((num_leds + offset) * 24) + 50);
    if (led_obj->pwm_data == NULL) {
        return LED_STRIP_MALLOC_FAILED;
    }
    return LED_STRIP_OK;
}

void led_strip_set_primary_LED(led_strip_t *led_obj, uint8_t LEDnum, uint8_t Red, uint8_t Green, uint8_t Blue) {
    led_obj->primary[LEDnum][0] = LEDnum;
    led_obj->primary[LEDnum][1] = Green;
    led_obj->primary[LEDnum][2] = Red;
    led_obj->primary[LEDnum][3] = Blue;
}

void led_strip_set_secondary_LED(led_strip_t *led_obj, uint8_t LEDnum, uint8_t Red, uint8_t Green, uint8_t Blue) {
    led_obj->secondary[LEDnum][0] = LEDnum;
    led_obj->secondary[LEDnum][1] = Green;
    led_obj->secondary[LEDnum][2] = Red;
    led_obj->secondary[LEDnum][3] = Blue;
}

void led_strip_set_LED(led_strip_t *led_obj, uint8_t LEDnum, uint8_t Red, uint8_t Green, uint8_t Blue) {
    led_obj->working[LEDnum][0] = LEDnum;
    led_obj->working[LEDnum][1] = Green;
    led_obj->working[LEDnum][2] = Red;
    led_obj->working[LEDnum][3] = Blue;
}

void led_strip_set_brightness(led_strip_t *led_obj, uint8_t brightness) {
#if USE_BRIGHTNESS
    if (brightness > 45) brightness = 45;
    uint8_t offset = (led_obj->sacrificial_led_flag * NUM_SACRIFICIAL_LED);
    uint8_t num_leds = led_obj->num_leds + offset;
    
    for (int i = 0; i < num_leds; i++) {
        led_obj->mod[i][0] = led_obj->working[i][0];
        for (int j = 1; j < 4; j++) {
            float angle = 90 - brightness;
            angle = angle*PI / 180;
            led_obj->mod[i][j] = (led_obj->working[i][j]) / (tan(angle));
        }
    }
#endif
}

void led_strip_clear(led_strip_t *led_obj) {
    uint8_t offset = (led_obj->sacrificial_led_flag * NUM_SACRIFICIAL_LED);
    for (int i = 0; i < led_obj->num_leds + led_obj->sacrificial_led_flag; i++) {
        led_strip_set_LED(led_obj, offset + i, 0, 0, 0);
    }
}

void led_strip_WS2812_send(led_strip_t *led_obj) {
    uint32_t indx = 0;
    uint32_t color;
    
    uint8_t offset = (led_obj->sacrificial_led_flag * NUM_SACRIFICIAL_LED);
    uint8_t num_leds = led_obj->num_leds + offset;

    for (int i = 0; i < num_leds; i++) {
#if USE_BRIGHTNESS
        color = ((led_obj->mod[i][1] << 16) | (led_obj->mod[i][2] << 8) | (led_obj->mod[i][3]));
#else
        color = ((led_obj->working[i][1] << 16) | (led_obj->working[i][2] << 8) | (led_obj->working[i][3]));
#endif
    
        for (int i = 23; i >= 0; i--) {
            if (color&(1 << i)) {
                led_obj->pwm_data[indx] = (uint16_t)led_obj->duty_high; // 2/3 of counter_period
            }
            else led_obj->pwm_data[indx] = (uint16_t)led_obj->duty_low; // 1/3 of counter_period
            indx++;
        }
    }
    
    for (int i = 0; i < 50; i++) {
        led_obj->pwm_data[indx] = 0;
        indx++;
    }
    
    while (!led_obj->data_sent_flag) {}
    ;
    led_obj->data_sent_flag = 0;
    HAL_TIM_PWM_Start_DMA(led_obj->htim, led_obj->channel, (uint32_t *)led_obj->pwm_data, indx);
    HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_RESET);
}

/*
 * @brief:  Fill the entire LED strip with a single color
 * 
 * @param:  led_obj: pointer to led_strip_t object
 * @param:  Red: Red value (0-255)
 * @param:  Green: Green value (0-255)
 * @param:  Blue: Blue value (0-255)
 * 
 * @return: None
 */

void led_strip_fill(led_strip_t *led_obj, uint8_t Red, uint8_t Green, uint8_t Blue)
{
    uint8_t offset = (led_obj->sacrificial_led_flag * NUM_SACRIFICIAL_LED);
    for (int i = 0; i < led_obj->num_leds + led_obj->sacrificial_led_flag; i++) {
        led_strip_set_LED(led_obj, offset + i, Red, Green, Blue);
    }
    
}

/*
 * @brief:  Fill a range of LEDs with a single color
 * 
 * @param:  led_obj: pointer to led_strip_t object
 * @param:  Red: Red value (0-255)
 * @param:  Green: Green value (0-255)
 * @param:  Blue: Blue value (0-255)
 * @param:  start: start LED number
 * @param:  end: end LED number
 * 
 * @return: None
 */

void led_strip_fill_range(led_strip_t *led_obj, uint8_t Red, uint8_t Green, uint8_t Blue, uint8_t start, uint8_t end)
{
    uint8_t offset = (led_obj->sacrificial_led_flag * NUM_SACRIFICIAL_LED);
    for (int i = start; i < end; i++) {
        led_strip_set_LED(led_obj, offset + i, Red, Green, Blue);
    }
}
