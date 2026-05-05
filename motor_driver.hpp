#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include <Arduino.h>

class MotorDriver {
private:
    int pwm_pin;
    int dir_pin;
    int pwm_freq;
    int pwm_res;
    int max_duty;
    int pwm_channel;

public:
    MotorDriver(int pwm_pin, int dir_pin, int pwm_freq = 20000, int pwm_res = 10, int channel = 0);
    void drive(float control_input);
};

#endif
