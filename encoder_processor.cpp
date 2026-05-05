#include "encoder_processor.hpp"
#include "esp_idf_version.h"

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#include "driver/pulse_cnt.h"
#else
#include "driver/pcnt.h"
#endif

EncoderProcessor::EncoderProcessor(int pinA, int pinB, float ppr)
    : pin_a(pinA), pin_b(pinB), pulses_per_revolution(ppr), total_count(0), prev_raw_count(0), pcnt_unit_handle(nullptr) {

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    pcnt_unit_handle_t unit = NULL;
    pcnt_unit_config_t unit_config = {};
    unit_config.low_limit = -32768;
    unit_config.high_limit = 32767;
    pcnt_new_unit(&unit_config, &unit);

    pcnt_glitch_filter_config_t filter_config = {};
    filter_config.max_glitch_ns = 1000;
    pcnt_unit_set_glitch_filter(unit, &filter_config);

    pcnt_chan_config_t chan_a_config = {};
    chan_a_config.edge_gpio_num = pinA;
    chan_a_config.level_gpio_num = pinB;
    pcnt_channel_handle_t chan_a = NULL;
    pcnt_new_channel(unit, &chan_a_config, &chan_a);

    pcnt_chan_config_t chan_b_config = {};
    chan_b_config.edge_gpio_num = pinB;
    chan_b_config.level_gpio_num = pinA;
    pcnt_channel_handle_t chan_b = NULL;
    pcnt_new_channel(unit, &chan_b_config, &chan_b);

    pcnt_channel_set_edge_action(chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    pcnt_channel_set_edge_action(chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    pcnt_unit_enable(unit);
    pcnt_unit_clear_count(unit);
    pcnt_unit_start(unit);
    
    pcnt_unit_handle = (void*)unit;

#else
    const pcnt_unit_t PCNT_UNIT = PCNT_UNIT_0;
    pcnt_config_t dev_config_a = {};
    dev_config_a.pulse_gpio_num = pinA;
    dev_config_a.ctrl_gpio_num = pinB;
    dev_config_a.lctrl_mode = PCNT_MODE_KEEP;
    dev_config_a.hctrl_mode = PCNT_MODE_REVERSE;
    dev_config_a.pos_mode = PCNT_COUNT_DEC;
    dev_config_a.neg_mode = PCNT_COUNT_INC;
    dev_config_a.counter_h_lim = 32767;
    dev_config_a.counter_l_lim = -32768;
    dev_config_a.unit = PCNT_UNIT;
    dev_config_a.channel = PCNT_CHANNEL_0;
    pcnt_unit_config(&dev_config_a);

    pcnt_config_t dev_config_b = {};
    dev_config_b.pulse_gpio_num = pinB;
    dev_config_b.ctrl_gpio_num = pinA;
    dev_config_b.lctrl_mode = PCNT_MODE_KEEP;
    dev_config_b.hctrl_mode = PCNT_MODE_REVERSE;
    dev_config_b.pos_mode = PCNT_COUNT_INC;
    dev_config_b.neg_mode = PCNT_COUNT_DEC;
    dev_config_b.counter_h_lim = 32767;
    dev_config_b.counter_l_lim = -32768;
    dev_config_b.unit = PCNT_UNIT;
    dev_config_b.channel = PCNT_CHANNEL_1;
    pcnt_unit_config(&dev_config_b);

    pcnt_set_filter_value(PCNT_UNIT, 100);
    pcnt_filter_enable(PCNT_UNIT);

    pcnt_counter_pause(PCNT_UNIT);
    pcnt_counter_clear(PCNT_UNIT);
    pcnt_counter_resume(PCNT_UNIT);
    
    pcnt_unit_handle = (void*)PCNT_UNIT;
#endif
}

void EncoderProcessor::update() {
    int raw_count = 0;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    pcnt_unit_get_count((pcnt_unit_handle_t)pcnt_unit_handle, &raw_count);
#else
    int16_t count16 = 0;
    pcnt_get_counter_value((pcnt_unit_t)(intptr_t)pcnt_unit_handle, &count16);
    raw_count = (int)count16;
#endif

    int16_t delta = (int16_t)(raw_count - prev_raw_count);
    total_count += delta;
    prev_raw_count = raw_count;
}

long long EncoderProcessor::getTotalCount() const {
    return total_count;
}

float EncoderProcessor::getAngleDegrees() const {
    return ((float)total_count / pulses_per_revolution) * 360.0f;
}
