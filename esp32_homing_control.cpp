#include <Arduino.h>
#include "motor_driver.hpp"
#include "encoder_processor.hpp"
#include "pid_controller.hpp"

// ハードウェアピン定義
const int PWM_PIN = 4;
const int DIR_PIN = 5;
const int ENCODER_PIN_A = 6;
const int ENCODER_PIN_B = 7;

// 制御パラメータ
const float CONTROL_PERIOD_SEC = 0.01f; // 10ms周期
const TickType_t CONTROL_PERIOD_MS = 10; 

// オブジェクトのインスタンス化（グローバル変数として保持）
MotorDriver* motor = nullptr;
EncoderProcessor* encoder = nullptr;
PIDController* pid = nullptr;

volatile float target_angle = 0.0f; // 原点(0度)を目標とする

// 原点復帰用のRTOSタスク
void homingControlTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true) {
        // 1. エンコーダの現在角度を取得
        encoder->update();
        float current_angle = encoder->getAngleDegrees();

        // 2. PID制御で必要なモーター出力を計算
        float control_input = pid->update(target_angle, current_angle);

        // 3. モーターへ出力
        motor->drive(control_input);

        // 4. 正確な周期(10ms)で待機
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

void setup() {
    Serial.begin(115200);

    // 各ハードウェア・アルゴリズムのオブジェクトを生成（動的確保）
    motor = new MotorDriver(PWM_PIN, DIR_PIN);
    encoder = new EncoderProcessor(ENCODER_PIN_A, ENCODER_PIN_B);
    pid = new PIDController(0.0137, 0.9380, 0.0, CONTROL_PERIOD_SEC); // Kp, Ki, Kd, dt
    // RTOSタスクの作成
    xTaskCreatePinnedToCore(
        homingControlTask,     // タスク関数
        "Homing_Control_Task", // タスク名
        4096,                  // スタックサイズ
        NULL,                  // パラメータ
        5,                     // 優先度
        NULL,                  // ハンドル
        1                      // 実行するコア(Core 1)
    );
}

void loop() {
    // メインループは他の処理（通信など）に使用可能
    delay(1000);
}
