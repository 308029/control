#include <Arduino.h>

const int PWM_PIN = 4;
const int DIR_PIN = 5;

const int PWM_FREQ = 20000;
const int PWM_RES = 10;
const int MAX_DUTY = (1 << PWM_RES) - 1;

const float Kp = 1.5f;
const float Ki = 0.05f;

const TickType_t CONTROL_PERIOD_MS = 10;

volatile float target_value = 0.0f;
volatile float current_value = 0.0f;

float readSensorDummy() {
    return current_value; 
}

void driveMotor(float control_input) {
    if (control_input >= 0) {
        digitalWrite(DIR_PIN, HIGH);
    } else {
        digitalWrite(DIR_PIN, LOW);
        control_input = -control_input;
    }

    if (control_input > 100.0f) {
        control_input = 100.0f;
    }
    
    uint32_t duty = (uint32_t)((control_input / 100.0f) * MAX_DUTY);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(PWM_PIN, duty);
#else
    ledcWrite(0, duty);
#endif
}

void piControlTask(void *pvParameters) {
    float integral = 0.0f;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true) {
        float measured_value = readSensorDummy();
        float error = target_value - measured_value;

        // アンチワインドアップ
        integral += error;
        const float MAX_INTEGRAL = 1000.0f;
        if (integral > MAX_INTEGRAL) integral = MAX_INTEGRAL;
        if (integral < -MAX_INTEGRAL) integral = -MAX_INTEGRAL;

        // PI制御計算
        float control_input = (Kp * error) + (Ki * integral);
        driveMotor(control_input);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

void setup() {
    Serial.begin(115200);

    pinMode(DIR_PIN, OUTPUT);
    digitalWrite(DIR_PIN, LOW);
    
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(PWM_PIN, PWM_FREQ, PWM_RES);
#else
    ledcSetup(0, PWM_FREQ, PWM_RES);
    ledcAttachPin(PWM_PIN, 0);
#endif

    xTaskCreatePinnedToCore(
        piControlTask,
        "PI_Control_Task",
        4096,
        NULL,
        5,
        NULL,
        1
    );
}

void loop() {
    delay(1000);
}
