#include "encoder.h"
#include "esp_idf_version.h"

// Maxon 201937 エンコーダの仕様
// CPT (Counts Per Turn) = 512
// 4逓倍（Quadrature）で読み取るため、1回転あたりのパルス数は 512 * 4 = 2048
const float PULSES_PER_REVOLUTION = 2048.0f;

static long long total_count = 0;
static int16_t prev_raw_count = 0;

// ESP32のハードウェアPCNTを利用して、CPU負荷ゼロでエンコーダをカウントします
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#include "driver/pulse_cnt.h"
static pcnt_unit_handle_t pcnt_unit = NULL;
#else
#include "driver/pcnt.h"
static const pcnt_unit_t PCNT_UNIT = PCNT_UNIT_0;
#endif

void encoderInit(int pinA, int pinB) {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    // ESP32 Arduino Core v3.x (ESP-IDF v5.x) 用
    pcnt_unit_config_t unit_config = {};
    unit_config.low_limit = -32768;
    unit_config.high_limit = 32767;
    pcnt_new_unit(&unit_config, &pcnt_unit);

    pcnt_glitch_filter_config_t filter_config = {};
    filter_config.max_glitch_ns = 1000;
    pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config);

    pcnt_chan_config_t chan_a_config = {};
    chan_a_config.edge_gpio_num = pinA;
    chan_a_config.level_gpio_num = pinB;
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a);

    pcnt_chan_config_t chan_b_config = {};
    chan_b_config.edge_gpio_num = pinB;
    chan_b_config.level_gpio_num = pinA;
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b);

    pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    pcnt_unit_enable(pcnt_unit);
    pcnt_unit_clear_count(pcnt_unit);
    pcnt_unit_start(pcnt_unit);

#else
    // ESP32 Arduino Core v2.x (ESP-IDF v4.x) 用
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
#endif

    prev_raw_count = 0;
    total_count = 0;
}

void encoderUpdate() {
    int raw_count = 0;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    pcnt_unit_get_count(pcnt_unit, &raw_count);
#else
    int16_t count16 = 0;
    pcnt_get_counter_value(PCNT_UNIT, &count16);
    raw_count = (int)count16;
#endif

    // 16bitハードウェアカウンタのオーバーフローに対処するため、
    // 差分（delta）を計算して64bit変数に積算します。
    // ※高速回転時でも制御ループ(10ms等)ごとに呼ばれれば取りこぼしません
    int16_t delta = (int16_t)(raw_count - prev_raw_count);
    total_count += delta;
    prev_raw_count = raw_count;
}

long long encoderGetTotalCount() {
    return total_count;
}

float encoderGetAngleDegrees() {
    // 角度(度) = (積算カウント / 1回転のカウント数) * 360
    return ((float)total_count / PULSES_PER_REVOLUTION) * 360.0f;
}
