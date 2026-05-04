#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

// エンコーダの初期化 (A相とB相のピン番号を指定)
void encoderInit(int pinA, int pinB);

// 制御周期ごとに呼び出してカウントの差分を積算する関数
void encoderUpdate();

// 現在の積算カウントを取得
long long encoderGetTotalCount();

// 現在の角度（度）を取得
float encoderGetAngleDegrees();

#endif
