#include <bits/stdc++.h>
#include "angular.hpp"
#include "control.hpp"
using namespace std;

int main() {
    double currentZ = 0.5;
    double currentY = 0.2;
    double currentX = 0.1;

    double wz = 0.01;
    double wy = 0.02;
    double wx = 0.03;
    double dt = 0.01;

    auto [nextZ, nextY, nextX] = integrateZyxEuler(currentZ, currentY, currentX, wz, wy, wx, dt);
    cout << fixed << setprecision(6);
    cout << "next yaw(z)=" << nextZ << " pitch(y)=" << nextY << " roll(x)=" << nextX << endl;

    // PID制御の例
    PIDController pid(1.0, 0.1, 0.01, dt);  // Kp=1.0, Ki=0.1, Kd=0.01
    double roll_command = pidControlRoll(nextX, pid);
    cout << "roll command (angular velocity): " << roll_command << endl;

    return 0;
}