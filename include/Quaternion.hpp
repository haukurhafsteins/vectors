#pragma once
#include <cmath>
#include <array>
#include "Vector3.hpp"

template <typename T>
class Quaternion
{
public:
    T w, x, y, z;

    // Default constructor (identity quaternion)
    Quaternion() : w(1), x(0), y(0), z(0) {}
    Quaternion(T w, T x, T y, T z) : w(w), x(x), y(y), z(z) {}

    // Create from axis-angle (axis must be normalized)
    static Quaternion fromAxisAngle(T ax, T ay, T az, T angleRad)
    {
        T halfAngle = angleRad * T(0.5);
        T s = std::sin(halfAngle);
        return Quaternion(std::cos(halfAngle), ax * s, ay * s, az * s);
    }

    // Norm (magnitude)
    T norm() const
    {
        return std::sqrt(w * w + x * x + y * y + z * z);
    }

    // Normalize
    Quaternion normalized() const
    {
        T n = norm();
        if (n == T(0))
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
        T n2 = w * w + x * x + y * y + z * z;
        if (n2 == T(0))
            return Quaternion(1, 0, 0, 0);
        return conjugate() * (T(1) / n2);
    }

    // Dot product
    T dot(const Quaternion &q) const
    {
        return w * q.w + x * q.x + y * q.y + z * q.z;
    }

    T angleBetween(const Quaternion<float>& q) const {
        T dotProduct = dot(q);
        // Clamp the dot product to the range [-1, 1] to avoid NaN from acos
        dotProduct = std::fmax(-1.0f, std::fmin(1.0f, dotProduct));
        return 2.0f * std::acos(dotProduct); // returns radians
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
    Quaternion operator*(T scalar) const
    {
        return Quaternion(w * scalar, x * scalar, y * scalar, z * scalar);
    }

    // Rotate a 3D vector
    Vector3<T> rotate(const Vector3<T> &v) const
    {
        Quaternion p(T(0), v.x, v.y, v.z);
        Quaternion result = (*this) * p * this->inverse();
        return Vector3<T>(result.x, result.y, result.z);
    }

    // Convert to 3x3 rotation matrix
    std::array<std::array<T, 3>, 3> toRotationMatrix() const
    {
        T xx = x * x, yy = y * y, zz = z * z;
        T xy = x * y, xz = x * z, yz = y * z;
        T wx = w * x, wy = w * y, wz = w * z;

        return {{{1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)},
                 {2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)},
                 {2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)}}};
    }

    static Quaternion fromAxisAngle(const Vector3<T> &axis, T angleRad)
    {
        T halfAngle = angleRad * T(0.5);
        T s = std::sin(halfAngle);
        return Quaternion(std::cos(halfAngle), axis.x * s, axis.y * s, axis.z * s);
    }

    // Static identity quaternion
    static Quaternion identity()
    {
        return Quaternion(T(1), T(0), T(0), T(0));
    }
};
