#include "motor_driver.hpp"
#include "esp_idf_version.h"

MotorDriver::MotorDriver(int pwm_pin, int dir_pin, int pwm_freq, int pwm_res, int channel)
    : pwm_pin(pwm_pin), dir_pin(dir_pin), pwm_freq(pwm_freq), pwm_res(pwm_res), pwm_channel(channel) {
    
    max_duty = (1 << pwm_res) - 1;

    pinMode(dir_pin, OUTPUT);
    digitalWrite(dir_pin, LOW);
    
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(pwm_pin, pwm_freq, pwm_res);
#else
    ledcSetup(pwm_channel, pwm_freq, pwm_res);
    ledcAttachPin(pwm_pin, pwm_channel);
#endif
}

void MotorDriver::drive(float control_input) {
    if (control_input >= 0) {
        digitalWrite(dir_pin, HIGH);
    } else {
        digitalWrite(dir_pin, LOW);
        control_input = -control_input;
    }

    if (control_input > 100.0f) {
        control_input = 100.0f;
    }
    
    uint32_t duty = (uint32_t)((control_input / 100.0f) * max_duty);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(pwm_pin, duty);
#else
    ledcWrite(pwm_channel, duty);
#endif
}
