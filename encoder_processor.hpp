#ifndef ENCODER_PROCESSOR_HPP
#define ENCODER_PROCESSOR_HPP

#include <Arduino.h>

class EncoderProcessor {
private:
    float pulses_per_revolution;
    long long total_count;
    int16_t prev_raw_count;
    int pin_a;
    int pin_b;
    
    // PCNTのハンドル。ESP-IDFのヘッダを隠蔽するためのvoidポインタ
    void* pcnt_unit_handle; 

public:
    // Maxon 201937 エンコーダ(512 CPT = 2048 pulses per revolution)
    EncoderProcessor(int pinA, int pinB, float ppr = 2048.0f);
    void update();
    long long getTotalCount() const;
    float getAngleDegrees() const;
};

#endif
