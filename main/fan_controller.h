#pragma once

#define FAN_PWM_GPIO     25  // Replace with your actual pin
#define FAN_PWM_FREQ_HZ  25000
#define FAN_PWM_TIMER    LEDC_TIMER_0
#define FAN_PWM_MODE     LEDC_HIGH_SPEED_MODE
#define FAN_PWM_CHANNEL  LEDC_CHANNEL_0
#define FAN_PWM_RES      LEDC_TIMER_8_BIT  // 0-255

void fans_init(void);
void fan_set_speed(float *temp_central, float *temp_exhaust);