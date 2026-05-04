#pragma once
#include <tuple>      // ← これを追加
#include <cmath>      // (数学関数用)
#include <algorithm>  // (clamp用)
using namespace std;
struct Quaternion {
    double w;
    double x;
    double y;
    double z;
};

Quaternion normalizeQuaternion(const Quaternion& q) {
    double norm = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (norm == 0.0) {
        return {1.0, 0.0, 0.0, 0.0};
    }
    return {q.w / norm, q.x / norm, q.y / norm, q.z / norm};
}

Quaternion quaternionMultiply(const Quaternion& a, const Quaternion& b) {
    return {
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    };
}

Quaternion eulerZYXToQuaternion(double z, double y, double x) {
    double cz = cos(z * 0.5);
    double sz = sin(z * 0.5);
    double cy = cos(y * 0.5);
    double sy = sin(y * 0.5);
    double cx = cos(x * 0.5);
    double sx = sin(x * 0.5);

    Quaternion q;
    q.w = cz * cy * cx + sz * sy * sx;
    q.x = cz * cy * sx - sz * sy * cx;
    q.y = cz * sy * cx + sz * cy * sx;
    q.z = sz * cy * cx - cz * sy * sx;
    return normalizeQuaternion(q);
}

tuple<double, double, double> quaternionToEulerZYX(const Quaternion& q_raw) {
    Quaternion q = normalizeQuaternion(q_raw);
    double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    double pitch = asin(min(max(sinp, -1.0), 1.0));
    double yaw = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    double roll = atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
    return {yaw, pitch, roll};
}

Quaternion integrateAngularVelocity(const Quaternion& q, double wz, double wy, double wx, double dt) {
    Quaternion omega = {0.0, wx, wy, wz};
    Quaternion q_dot = quaternionMultiply(q, omega);
    Quaternion q_next = {
        q.w + 0.5 * q_dot.w * dt,
        q.x + 0.5 * q_dot.x * dt,
        q.y + 0.5 * q_dot.y * dt,
        q.z + 0.5 * q_dot.z * dt,
    };
    return normalizeQuaternion(q_next);
}

tuple<double, double, double> integrateZyxEuler(double currentZ, double currentY, double currentX, double wz, double wy, double wx, double dt) {
    Quaternion q = eulerZYXToQuaternion(currentZ, currentY, currentX);
    Quaternion q_next = integrateAngularVelocity(q, wz, wy, wx, dt);
    return quaternionToEulerZYX(q_next);
}
