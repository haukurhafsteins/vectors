#pragma once
#include <math.h>
#include <array>

class Quaternion
{
public:
    // Components
    float w, x, y, z;

    // Constructors
    Quaternion() : w(1), x(0), y(0), z(0) {}
    Quaternion(float w, float x, float y, float z)
        : w(w), x(x), y(y), z(z) {}

    // Create from axis-angle (axis must be normalized)
    static Quaternion fromAxisAngle(float ax, float ay, float az, float angleRad)
    {
        float halfAngle = angleRad * 0.5f;
        float s = std::sin(halfAngle);
        return Quaternion(std::cos(halfAngle), ax * s, ay * s, az * s);
    }

    // Quaternion magnitude
    float norm() const
    {
        return std::sqrt(w * w + x * x + y * y + z * z);
    }

    // Normalize
    Quaternion normalized() const
    {
        float n = norm();
        if (n == 0.0f)
            return Quaternion(1, 0, 0, 0);
        return Quaternion(w / n, x / n, y / n, z / n);
    }

    // Conjugate
    Quaternion conjugate() const
    {
        return Quaternion(w, -x, -y, -z);
    }

    // Inverse
    Quaternion inverse() const
    {
        float n2 = w * w + x * x + y * y + z * z;
        if (n2 == 0.0f)
            return Quaternion(1, 0, 0, 0);
        return conjugate() * (1.0f / n2);
    }

    float dot(const Quaternion &q) const
    {
        return w * q.w + x * q.x + y * q.y + z * q.z;
    }

    // Quaternion multiplication
    Quaternion operator*(const Quaternion &q) const
    {
        return Quaternion(
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w);
    }

    // Scalar multiplication
    Quaternion operator*(float scalar) const
    {
        return Quaternion(w * scalar, x * scalar, y * scalar, z * scalar);
    }

    // Rotate a vector
    std::array<float, 3> rotate(const std::array<float, 3> &v) const
    {
        Quaternion p(0, v[0], v[1], v[2]);
        Quaternion result = (*this) * p * this->inverse();
        return {result.x, result.y, result.z};
    }

    // To 3x3 rotation matrix
    std::array<std::array<float, 3>, 3> toRotationMatrix() const
    {
        float xx = x * x, yy = y * y, zz = z * z;
        float xy = x * y, xz = x * z, yz = y * z;
        float wx = w * x, wy = w * y, wz = w * z;

        return {{{1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)},
                 {2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)},
                 {2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)}}};
    }

    // Identity quaternion
    static Quaternion identity()
    {
        return Quaternion(1, 0, 0, 0);
    }
};
