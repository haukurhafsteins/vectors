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
    T magnitude() const
    {
        return std::sqrt(w * w + x * x + y * y + z * z);
    }

    // Normalize
    Quaternion normalized() const
    {
        T n = magnitude();
        if (n == T(0))
            return Quaternion(1, 0, 0, 0);
        return Quaternion(w / n, x / n, y / n, z / n);
    }

    Quaternion& normalize()
    {
        T n = magnitude();
        if (n == T(0))
        {
            w = 1; x = 0; y = 0; z = 0;
        }
        else
        {
            w /= n; x /= n; y /= n; z /= n;
        }
        return *this;
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

    T angleBetween(const Quaternion<T>& q) const {
        T dotProduct = dot(q);
        // Clamp the dot product to the range [-1, 1] to avoid NaN from acos
        dotProduct = std::fmax(-1.0f, std::fmin(1.0f, dotProduct));
        return 2.0f * std::acos(dotProduct); // returns radians
    }

    T angleBetweenDeg(const Quaternion<T>& q) const {
        return angleBetween(q) * (180.0f / M_PI); // Convert radians to degrees
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

    // Convert to axis-angle representation
    Vector3<T> toAxisAngle() const
    {
        T angle = std::acos(w) * T(2); // Angle in radians
        T s = std::sqrt(x * x + y * y + z * z);
        if (s < T(1e-6))                                // Avoid division by zero
            return Vector3<T>(1, 0, 0) * angle;         // Default axis if no rotation
        return Vector3<T>(x / s, y / s, z / s) * angle; // Axis * angle
    }

    // Check if q is within margin radians of this quaternion
    bool isWithin(const Quaternion &q, T marginRad) const
    {
        Quaternion qrel = this->inverse() * q;
        Vector3<T> aa = qrel.toAxisAngle(); // Axis * angle (in radians)
        T angle = aa.magnitude();           // Total rotation angle
        return angle <= marginRad;
    }

    // Convert quaternion and gravity vector into yaw, pitch, roll (in degrees)
    Vector3<T> yawPitchRoll(const Vector3<T> &g) const{
        float pitch = std::atan2(-g.x, std::sqrt(g.y * g.y + g.z * g.z)) * (180.0f / M_PI);
        float roll  = std::atan2(g.y, g.z) * (180.0f / M_PI);

        // Extract yaw from quaternion
        float siny_cosp = 2.0f * (w * z + x * y);
        float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
        float yaw = std::atan2(siny_cosp, cosy_cosp) * (180.0f / M_PI);

        return { yaw, pitch, roll };
    }

        // Estimate a reference vector in the ground plane (perpendicular to gravity) using the reference quaternion
    Vector3<T> referenceInPlane(const Vector3<T> &gravity) 
    {
        // Define forward vector in local space
        Vector3<T> forward_local = {1.0f, 0.0f, 0.0f};

        // Rotate forward vector by reference quaternion
        Quaternion<T> f = {forward_local.x, forward_local.y, forward_local.z, 0.0f};
        Quaternion<T> q_conj = {-x, -y, -z, w};

        Quaternion<T> tmp = {
            w * f.x + y * f.z - z * f.y,
            w * f.y + z * f.x - x * f.z,
            w * f.z + x * f.y - y * f.x,
            -x * f.x - y * f.y - z * f.z
        };

        Quaternion<T> rotated = {
            tmp.w * q_conj.x + tmp.x * q_conj.w + tmp.y * q_conj.z - tmp.z * q_conj.y,
            tmp.w * q_conj.y + tmp.y * q_conj.w + tmp.z * q_conj.x - tmp.x * q_conj.z,
            tmp.w * q_conj.z + tmp.z * q_conj.w + tmp.x * q_conj.y - tmp.y * q_conj.x,
            tmp.w * q_conj.w - tmp.x * q_conj.x - tmp.y * q_conj.y - tmp.z * q_conj.z
        };

        // Subtract gravity component to project into plane
        float g_mag = std::sqrt(gravity.x * gravity.x + gravity.y * gravity.y + gravity.z * gravity.z);
        if (g_mag < TOLERANCE) return {0.0f, 0.0f, 0.0f};
        Vector3<T> g_unit = { gravity.x / g_mag, gravity.y / g_mag, gravity.z / g_mag };

        float dot = rotated.x * g_unit.x + rotated.y * g_unit.y + rotated.z * g_unit.z;
        Vector3<T> proj = {
            rotated.x - dot * g_unit.x,
            rotated.y - dot * g_unit.y,
            rotated.z - dot * g_unit.z
        };

        float proj_mag = std::sqrt(proj.x * proj.x + proj.y * proj.y + proj.z * proj.z);
        if (proj_mag < TOLERANCE) return {0.0f, 0.0f, 0.0f};

        return {proj.x / proj_mag, proj.y / proj_mag, proj.z / proj_mag};
    }

    Quaternion<T> relative(const Quaternion<T> &q2) {
        Quaternion<T> q1_inv = { -x, -y, -z, w }; // Conjugate (assuming unit quaternion)
        Quaternion<T> q_rel;
        q_rel.w = q1_inv.w*q2.w - q1_inv.x*q2.x - q1_inv.y*q2.y - q1_inv.z*q2.z;
        q_rel.x = q1_inv.w*q2.x + q1_inv.x*q2.w + q1_inv.y*q2.z - q1_inv.z*q2.y;
        q_rel.y = q1_inv.w*q2.y - q1_inv.x*q2.z + q1_inv.y*q2.w + q1_inv.z*q2.x;
        q_rel.z = q1_inv.w*q2.z + q1_inv.x*q2.y - q1_inv.y*q2.x + q1_inv.z*q2.w;
        q_rel.normalize();
        return q_rel;
    }
    float signedProjectedRotationDeg(const Quaternion<T> &q2, const Vector3<T> &gravity, const Vector3<T> &reference_in_plane) {
        Quaternion<T> q_rel = relative(q2);
        float angle_rad = 2.0f * std::atan2(
            std::sqrt(q_rel.x * q_rel.x + q_rel.y * q_rel.y + q_rel.z * q_rel.z),
            q_rel.w);

        float norm = q_rel.magnitude();
        if (norm < TOLERANCE) return 0.0f;

        Vector3<T> rot_axis = { q_rel.x / norm, q_rel.y / norm, q_rel.z / norm };

        // Normalize gravity vector
        float g_mag = std::sqrt(gravity.x * gravity.x + gravity.y * gravity.y + gravity.z * gravity.z);
        if (g_mag < TOLERANCE) return 0.0f;

        Vector3<T> g_unit = { gravity.x / g_mag, gravity.y / g_mag, gravity.z / g_mag };

        // Project rotation axis onto plane perpendicular to gravity
        float dot_g = rot_axis.x * g_unit.x + rot_axis.y * g_unit.y + rot_axis.z * g_unit.z;
        Vector3<T> proj = {
            rot_axis.x - dot_g * g_unit.x,
            rot_axis.y - dot_g * g_unit.y,
            rot_axis.z - dot_g * g_unit.z
        };

        float proj_mag = std::sqrt(proj.x * proj.x + proj.y * proj.y + proj.z * proj.z);
        if (proj_mag < TOLERANCE) return 0.0f;

        // Normalize projected vector
        proj.x /= proj_mag;
        proj.y /= proj_mag;
        proj.z /= proj_mag;

        // Compute signed angle using cross product with reference in plane
        Vector3<T> ref = reference_in_plane; // Assumed to be normalized and in the ground plane
        float sign = (ref.x * proj.y - ref.y * proj.x + ref.z * proj.z) > 0.0f ? 1.0f : -1.0f;

        return sign * angle_rad * proj_mag * (180.0f / M_PI);
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

    static constexpr float TOLERANCE = 1e-2f;
};
