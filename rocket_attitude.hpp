#ifndef ROCKET_ATTITUDE_HPP
#define ROCKET_ATTITUDE_HPP

#include <tuple>
#include <cmath>
#include <algorithm>

struct Quaternion {
    double w, x, y, z;
};

// 機体の姿勢を管理・更新するクラス
class RocketAttitude {
private:
    Quaternion q; // 現在の姿勢（クォータニオン）

    Quaternion normalize(const Quaternion& q_raw) const {
        double norm = std::sqrt(q_raw.w * q_raw.w + q_raw.x * q_raw.x + q_raw.y * q_raw.y + q_raw.z * q_raw.z);
        if (norm == 0.0) return {1.0, 0.0, 0.0, 0.0};
        return {q_raw.w / norm, q_raw.x / norm, q_raw.y / norm, q_raw.z / norm};
    }

    Quaternion multiply(const Quaternion& a, const Quaternion& b) const {
        return {
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        };
    }

public:
    RocketAttitude(double yaw_rad = 0.0, double pitch_rad = 0.0, double roll_rad = 0.0) {
        setFromEulerZYX(yaw_rad, pitch_rad, roll_rad);
    }

    // オイラー角(ZYX)から姿勢をセットする
    void setFromEulerZYX(double yaw, double pitch, double roll) {
        double cz = std::cos(yaw * 0.5);
        double sz = std::sin(yaw * 0.5);
        double cy = std::cos(pitch * 0.5);
        double sy = std::sin(pitch * 0.5);
        double cx = std::cos(roll * 0.5);
        double sx = std::sin(roll * 0.5);

        q.w = cz * cy * cx + sz * sy * sx;
        q.x = cz * cy * sx - sz * sy * cx;
        q.y = cz * sy * cx + sz * cy * sx;
        q.z = sz * cy * cx - cz * sy * sx;
        q = normalize(q);
    }

    // 角速度(wx, wy, wz)を与えて姿勢を積分(更新)する
    void update(double wx, double wy, double wz, double dt) {
        Quaternion omega = {0.0, wx, wy, wz};
        Quaternion q_dot = multiply(q, omega);
        q.w += 0.5 * q_dot.w * dt;
        q.x += 0.5 * q_dot.x * dt;
        q.y += 0.5 * q_dot.y * dt;
        q.z += 0.5 * q_dot.z * dt;
        q = normalize(q);
    }

    // 現在のオイラー角を取得する tuple<yaw, pitch, roll>
    std::tuple<double, double, double> getEulerZYX() const {
        double sinp = 2.0 * (q.w * q.y - q.z * q.x);
        double pitch = std::asin(std::min(std::max(sinp, -1.0), 1.0));
        double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        double roll = std::atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y));
        return {yaw, pitch, roll};
    }

    Quaternion getQuaternion() const { return q; }
};

#endif
