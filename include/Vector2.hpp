#pragma once
#include <cstdio>
#include <cmath>

struct Vector2 {
    float x;
    float y;
    Vector2() : x(0), y(0) {}
    Vector2(float x, float y) : x(x), y(y) {}

    void print() const {
        printf("Vector2(%.3f, %.3f)\n", x, y);
    }

    // Magnitude
    float magnitude() const {
        return std::sqrt(x * x + y * y);
    }

    // Normalize
    Vector2 normalized() const {
        float n = magnitude();
        if (n == 0) return Vector2(0, 0);
        return *this / n;
    }

    // Dot product
    float dot(const Vector2& v) const {
        return x * v.x + y * v.y;
    }

    // Operator overloads
    Vector2 operator+(const Vector2& v) const {
        return Vector2(x + v.x, y + v.y);
    }

    Vector2 operator-(const Vector2& v) const {
        return Vector2(x - v.x, y - v.y);
    }

    Vector2 operator*(float scalar) const {
        return Vector2(x * scalar, y * scalar);
    }

    Vector2 operator/(float scalar) const {
        if (scalar == 0) return Vector2(0, 0);
        return Vector2(x / scalar, y / scalar);
    }

    Vector2& operator+=(const Vector2& v) {
        x += v.x; y += v.y;
        return *this;
    }
    Vector2& operator-=(const Vector2& v) {
        x -= v.x; y -= v.y;
        return *this;
    }
    Vector2& operator*=(float scalar) {
        x *= scalar; y *= scalar;
        return *this; 
    }
    Vector2& operator/=(float scalar) {
        if (scalar == 0) {
            x = 0; y = 0;
        } else {
            x /= scalar; y /= scalar;
        }
        return *this;
    }
};