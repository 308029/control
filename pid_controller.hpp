#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

class PIDController {
private:
    double Kp, Ki, Kd;
    double integral;
    double prev_error;
    double dt;
    double max_integral;

public:
    PIDController(double kp, double ki, double kd, double dt_val, double max_integral_val = 1000.0) 
        : Kp(kp), Ki(ki), Kd(kd), integral(0.0), prev_error(0.0), dt(dt_val), max_integral(max_integral_val) {}

    // 目標値(target)と現在値(current)を与えて制御出力を計算
    double update(double target, double current) {
        double error = target - current;
        integral += error * dt;
        
        // アンチワインドアップ (積分器の飽和防止)
        if (integral > max_integral) integral = max_integral;
        if (integral < -max_integral) integral = -max_integral;

        double derivative = (error - prev_error) / dt;
        double output = Kp * error + Ki * integral + Kd * derivative;
        
        prev_error = error;
        return output;
    }
    
    // 制御器の状態をリセット
    void reset() {
        integral = 0.0;
        prev_error = 0.0;
    }
};

#endif