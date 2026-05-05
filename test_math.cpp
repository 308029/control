#include <bits/stdc++.h>
#include "rocket_attitude.hpp"
#include "pid_controller.hpp"
using namespace std;

int main() {
    double dt = 0.01;

    // 姿勢オブジェクトの作成
    RocketAttitude attitude(0.5, 0.2, 0.1); // 初期角度(Yaw, Pitch, Roll) ラジアン

    double wz = 0.01;
    double wy = 0.02;
    double wx = 0.03;

    // 姿勢の更新
    attitude.update(wx, wy, wz, dt);

    auto [nextZ, nextY, nextX] = attitude.getEulerZYX();
    cout << fixed << setprecision(6);
    cout << "next yaw(z)=" << nextZ << " pitch(y)=" << nextY << " roll(x)=" << nextX << endl;

    // PID制御の例
    PIDController pid(1.0, 0.1, 0.01, dt);  // Kp=1.0, Ki=0.1, Kd=0.01
    
    double target_roll = 0.0;
    double roll_command = pid.update(target_roll, nextX);
    cout << "roll command (angular velocity): " << roll_command << endl;

    return 0;
}