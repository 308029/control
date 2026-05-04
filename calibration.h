#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <Arduino.h>

const int FFT_SIZE = 128; // FFTのサンプル数（2の累乗である必要があります）

class ImuFilter {
private:
    float alpha[6];          // 各軸のローパスフィルタ（EMA）係数
    float filtered_data[6];  // 各軸のフィルタリングされた最新データ
    
    // キャリブレーション用バッファ
    float calib_buffer[6][FFT_SIZE];
    int sample_count;
    bool is_calibrated;
    float sampling_freq;

    // 内部処理用関数
    void calculateFFTAndFilter(int axis);
    void computeFFT(float* vReal, float* vImag, uint16_t samples);

public:
    // コンストラクタ（IMUのサンプリング周波数Hzを指定）
    ImuFilter(float sampling_rate_hz);

    // 1. キャリブレーションデータを追加する
    // 返り値が false の間は蓄積中。true になったら解析とフィルタ構築完了。
    bool addCalibrationData(float ax, float ay, float az, float gx, float gy, float gz);

    // 2. リアルタイムフィルタリング
    // 最新の6軸データを与えると、フィルタリングされたデータを引数の参照先に返す
    void updateFilter(float ax, float ay, float az, float gx, float gy, float gz,
                      float& out_ax, float& out_ay, float& out_az, 
                      float& out_gx, float& out_gy, float& out_gz);
                      
    // キャリブレーションが完了しているか確認
    bool isCalibrated() const { return is_calibrated; }
};

#endif
