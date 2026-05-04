#include <bits/stdc++.h>
using namespace std;
// PID制御器構造体
struct PIDController {
    double Kp;
    double Ki;
    double Kd;
    double integral;
    double prev_error;
    double dt;

    PIDController(double kp, double ki, double kd, double dt_val) : Kp(kp), Ki(ki), Kd(kd), integral(0.0), prev_error(0.0), dt(dt_val) {}

    double update(double error) {
        integral += error * dt;
        double derivative = (error - prev_error) / dt;
        double output = Kp * error + Ki * integral + Kd * derivative;
        prev_error = error;
        return output;
    }
};

// ロール角を0にキープするためのPID制御
// 現在のroll角を受け取り、ロール軸周りの角速度コマンドを返す
double pidControlRoll(double current_roll, PIDController& pid) {
    double target_roll = 0.0;
    double error = target_roll - current_roll;
    return pid.update(error);
}