#include <stdio.h>
#include "fans.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include <math.h>

#define FAN_PWM_PIN 14
#define INTAKE_FAN_TAC_PIN 17
#define EXHAUST_FAN_TAC_PIN 22

#define FAN_SPEED_MODE LEDC_LOW_SPEED_MODE
#define FAN_LEDC_CHANNEL LEDC_CHANNEL_1


#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif



/**
 * Clamp an integer between two values
 * @param value int Value to clamp
 * @param min_val int Minimum value
 * @param max_val int Maximum value
 * @return The clamped integer
 */
static int clamp_int(int value, int min_val, int max_val) {
    return MIN(MAX(value, min_val), max_val);
}

static void tach_isr() {

}

void fans_init() {
    ledc_timer_config_t ledc_timer = {
            .speed_mode = FAN_SPEED_MODE,
            .duty_resolution = LEDC_TIMER_8_BIT,
            .timer_num = LEDC_TIMER_1,
            .freq_hz = 25000,
            .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    ledc_channel_config_t ledc_channel = {
            .speed_mode = FAN_SPEED_MODE,
            .channel = LEDC_CHANNEL_1,
            .timer_sel = LEDC_TIMER_1,
            .intr_type = LEDC_INTR_DISABLE,
            .gpio_num = FAN_PWM_PIN,
            .duty = 0,
            .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    // Set fans to 100% immediately
    ledc_set_duty(FAN_SPEED_MODE, FAN_LEDC_CHANNEL, 255);
    ledc_update_duty(FAN_SPEED_MODE, FAN_LEDC_CHANNEL);


    gpio_config_t tac_conf = {
            .intr_type = GPIO_INTR_NEGEDGE,
            .mode = GPIO_MODE_INPUT,
            .pin_bit_mask = (1ULL << INTAKE_FAN_TAC_PIN) | (1ULL << EXHAUST_FAN_TAC_PIN),
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE
    };
    gpio_config(&tac_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(INTAKE_FAN_TAC_PIN, tach_isr, NULL);
    gpio_isr_handler_add(EXHAUST_FAN_TAC_PIN, tach_isr, NULL);

}


/**
 * Set the speed of the fans
 * @param speed uint8_t percent to set fans, from 0 to 100
 */
void fans_set_speed(uint8_t percent) {
    percent = clamp_int(percent, 0, 100);
    int speed = (int)round(percent * 2.55);
    ledc_set_duty(FAN_SPEED_MODE, FAN_LEDC_CHANNEL, speed);
    ledc_update_duty(FAN_SPEED_MODE, FAN_LEDC_CHANNEL);
}

/**
 * Set the duty cycle of the fans
 * @param duty_cycle uint8_t duty cycle to set fans to, from 0 to 255
 */
void fans_set_duty_cycle(uint8_t duty_cycle) {
    ledc_set_duty(FAN_SPEED_MODE, FAN_LEDC_CHANNEL, duty_cycle);
    ledc_update_duty(FAN_SPEED_MODE, FAN_LEDC_CHANNEL);
}

