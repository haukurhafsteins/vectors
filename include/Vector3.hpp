#pragma once
#include <cmath>
#include <initializer_list>
#include <stdexcept>

template <typename T>
class Vector3 {
public:
    T x, y, z;

    // Constructors
    Vector3() : x(0), y(0), z(0) {}
    Vector3(T x, T y, T z) : x(x), y(y), z(z) {}
    
    // Construct from initializer list
    Vector3(std::initializer_list<T> list) {
        auto it = list.begin();
        if (list.size() != 3) throw std::invalid_argument("Vector3 must be initialized with 3 elements");
        x = *it++;
        y = *it++;
        z = *it;
    }

    // Vector length (magnitude)
    T magnitude() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    // Normalize
    Vector3 normalized() const {
        T n = magnitude();
        if (n == T(0)) return Vector3(0, 0, 0);
        return *this / n;
    }

    // Dot product
    T dot(const Vector3& v) const {
        return x * v.x + y * v.y + z * v.z;
    }

    // Cross product
    Vector3 cross(const Vector3& v) const {
        return Vector3(
            y * v.z - z * v.y,
            z * v.x - x * v.z,
            x * v.y - y * v.x
        );
    }

    // Operator overloads
    Vector3 operator+(const Vector3& v) const {
        return Vector3(x + v.x, y + v.y, z + v.z);
    }

    Vector3 operator-(const Vector3& v) const {
        return Vector3(x - v.x, y - v.y, z - v.z);
    }

    Vector3 operator*(T scalar) const {
        return Vector3(x * scalar, y * scalar, z * scalar);
    }

    Vector3 operator/(T scalar) const {
        return Vector3(x / scalar, y / scalar, z / scalar);
    }

    Vector3& operator+=(const Vector3& v) {
        x += v.x; y += v.y; z += v.z;
        return *this;
    }

    Vector3& operator-=(const Vector3& v) {
        x -= v.x; y -= v.y; z -= v.z;
        return *this;
    }

    Vector3& operator*=(T scalar) {
        x *= scalar; y *= scalar; z *= scalar;
        return *this;
    }

    Vector3& operator/=(T scalar) {
        x /= scalar; y /= scalar; z /= scalar;
        return *this;
    }

    // Convert to std::array<T, 3>
    std::array<T, 3> toArray() const {
        return {x, y, z};
    }

    // Zero vector
    static Vector3 zero() {
        return Vector3(0, 0, 0);
    }

    // Unit vector
    static Vector3 unitX() { return Vector3(1, 0, 0); }
    static Vector3 unitY() { return Vector3(0, 1, 0); }
    static Vector3 unitZ() { return Vector3(0, 0, 1); }
};

// Scalar * vector support
template <typename T>
Vector3<T> operator*(T scalar, const Vector3<T>& v) {
    return Vector3<T>(v.x * scalar, v.y * scalar, v.z * scalar);
}
