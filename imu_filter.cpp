#include "imu_filter.h"
#include <math.h>

ImuFilter::ImuFilter(float sampling_rate_hz) {
    sampling_freq = sampling_rate_hz;
    sample_count = 0;
    is_calibrated = false;
    for(int i = 0; i < 6; i++) {
        alpha[i] = 1.0f; // 初期値（フィルタなし。入力がそのまま出力される）
        filtered_data[i] = 0.0f;
    }
}

bool ImuFilter::addCalibrationData(float ax, float ay, float az, float gx, float gy, float gz) {
    if (is_calibrated) return true;

    if (sample_count < FFT_SIZE) {
        calib_buffer[0][sample_count] = ax;
        calib_buffer[1][sample_count] = ay;
        calib_buffer[2][sample_count] = az;
        calib_buffer[3][sample_count] = gx;
        calib_buffer[4][sample_count] = gy;
        calib_buffer[5][sample_count] = gz;
        sample_count++;
    }

    if (sample_count >= FFT_SIZE) {
        // バッファが規定数（FFT_SIZE）まで溜まったら、全軸についてFFT解析し、フィルタを構築する
        for (int i = 0; i < 6; i++) {
            calculateFFTAndFilter(i);
        }
        is_calibrated = true;
        return true;
    }

    return false;
}

// 外部ライブラリ不要な軽量バタフライ演算（Cooley-Tukey FFTアルゴリズム）
void ImuFilter::computeFFT(float* vReal, float* vImag, uint16_t samples) {
    // 1. ビットリバース処理
    uint16_t j = 0;
    for (uint16_t i = 0; i < samples - 1; i++) {
        if (i < j) {
            float tempR = vReal[i];
            float tempI = vImag[i];
            vReal[i] = vReal[j];
            vImag[i] = vImag[j];
            vReal[j] = tempR;
            vImag[j] = tempI;
        }
        uint16_t k = samples >> 1;
        while (k <= j) {
            j -= k;
            k >>= 1;
        }
        j += k;
    }

    // 2. バタフライ演算
    for (uint16_t halfSize = 1; halfSize < samples; halfSize *= 2) {
        float phaseShiftStepR = cos(-PI / halfSize);
        float phaseShiftStepI = sin(-PI / halfSize);
        float currentPhaseShiftR = 1.0;
        float currentPhaseShiftI = 0.0;
        
        for (uint16_t fftStep = 0; fftStep < halfSize; fftStep++) {
            for (uint16_t i = fftStep; i < samples; i += 2 * halfSize) {
                float tempR = currentPhaseShiftR * vReal[i + halfSize] - currentPhaseShiftI * vImag[i + halfSize];
                float tempI = currentPhaseShiftR * vImag[i + halfSize] + currentPhaseShiftI * vReal[i + halfSize];
                vReal[i + halfSize] = vReal[i] - tempR;
                vImag[i + halfSize] = vImag[i] - tempI;
                vReal[i] += tempR;
                vImag[i] += tempI;
            }
            float tmpR = currentPhaseShiftR;
            currentPhaseShiftR = tmpR * phaseShiftStepR - currentPhaseShiftI * phaseShiftStepI;
            currentPhaseShiftI = tmpR * phaseShiftStepI + currentPhaseShiftI * phaseShiftStepR;
        }
    }
}

void ImuFilter::calculateFFTAndFilter(int axis) {
    float vReal[FFT_SIZE];
    float vImag[FFT_SIZE];

    // 平均値（DC成分: 0Hzの成分）を計算
    float sum = 0;
    for (int i = 0; i < FFT_SIZE; i++) {
        sum += calib_buffer[axis][i];
    }
    float mean = sum / FFT_SIZE;

    // FFTにかける前に平均値を引き、DC成分を除去する
    for (int i = 0; i < FFT_SIZE; i++) {
        vReal[i] = calib_buffer[axis][i] - mean;
        vImag[i] = 0.0f;
    }

    // フーリエ変換を実行
    computeFFT(vReal, vImag, FFT_SIZE);

    // 振幅スペクトルを計算し、最も強いノイズのピーク周波数（Hz）を探す
    // bin=0 は除去済みなので無視。ナイキスト周波数（FFT_SIZE/2）まで探索。
    float max_magnitude = 0;
    int max_bin = 1;
    for (int i = 1; i < FFT_SIZE / 2; i++) {
        float magnitude = sqrt(vReal[i] * vReal[i] + vImag[i] * vImag[i]);
        if (magnitude > max_magnitude) {
            max_magnitude = magnitude;
            max_bin = i;
        }
    }

    // 解析された最大のノイズピークの周波数(Hz)
    float noise_freq = (float)max_bin * sampling_freq / FFT_SIZE;

    // --- ここからローパスフィルタの設計（算出） ---
    // ノイズ周波数を確実に減衰させるため、カットオフ周波数をノイズピークの0.3倍に設定
    float cutoff_freq = noise_freq * 0.3f; 
    
    // システムの安定性のための安全装置（高すぎたり低すぎたりするのを防ぐ）
    if (cutoff_freq > 20.0f) cutoff_freq = 20.0f; // ノイズが高周波すぎても、カットオフ上限は20Hz程度にする
    if (cutoff_freq < 2.0f)  cutoff_freq = 2.0f;  // 低すぎるとフィルタの遅延が激しくなるので下限を設ける

    // 1次ローパスフィルタ(EMA: 指数加重移動平均)の係数アルファを計算
    // 計算式: RC = 1 / (2 * PI * fc), dt = 1 / fs, alpha = dt / (RC + dt)
    float dt = 1.0f / sampling_freq;
    float rc = 1.0f / (2.0f * PI * cutoff_freq);
    alpha[axis] = dt / (rc + dt);

    // フィルタの初期値として、キャリブレーション中の平均値をセットしておく
    filtered_data[axis] = mean;
}

void ImuFilter::updateFilter(float ax, float ay, float az, float gx, float gy, float gz,
                             float& out_ax, float& out_ay, float& out_az, 
                             float& out_gx, float& out_gy, float& out_gz) {
    
    // 入力を配列化してループで処理しやすくする
    float raw[6] = {ax, ay, az, gx, gy, gz};
    
    // 1次IIRローパスフィルタを適用（遅延がほぼゼロで高速に処理されます）
    for (int i = 0; i < 6; i++) {
        filtered_data[i] = alpha[i] * raw[i] + (1.0f - alpha[i]) * filtered_data[i];
    }

    // 参照引数に出力値をセット
    out_ax = filtered_data[0];
    out_ay = filtered_data[1];
    out_az = filtered_data[2];
    out_gx = filtered_data[3];
    out_gy = filtered_data[4];
    out_gz = filtered_data[5];
}
