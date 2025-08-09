#pragma once
#include <cmath>
#include <array>
#include "Vector3.hpp"

template <typename T>
class Quaternion
{
public:
    T x, y, z, w;

    // Default constructor (identity quaternion)
    Quaternion() : x(0), y(0), z(0), w(1) {}
    Quaternion(T x, T y, T z, T w) : x(x), y(y), z(z), w(w) {}

    // Create from axis-angle (axis must be normalized)
    static Quaternion fromAxisAngle(T ax, T ay, T az, T angleRad)
    {
        T halfAngle = angleRad * T(0.5);
        T s = std::sin(halfAngle);
        return Quaternion(ax * s, ay * s, az * s, std::cos(halfAngle));
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
        return Quaternion(x / n, y / n, z / n, w / n);
    }

    void null()
    {
        x = 0;
        y = 0;
        z = 0;
        w = 1;
    }

    Quaternion &normalize()
    {
        T n = magnitude();
        if (n == T(0))
        {
            null();
        }
        else
        {
            x /= n;
            y /= n;
            z /= n;
            w /= n;
        }
        return *this;
    }

    // Conjugate
    Quaternion conjugate() const
    {
        return Quaternion(-x, -y, -z, w);
    }

    // Inverse
    Quaternion inverse() const
    {
        T n2 = w * w + x * x + y * y + z * z;
        if (n2 == T(0))
        {
            // Norm squared is zero; inverse is undefined. Returning identity quaternion as fallback.
            // Alternatively, you may throw an exception here:
            // throw std::runtime_error("Cannot invert a zero-norm quaternion.");
            return Quaternion(0, 0, 0, 1);
        }
        return conjugate() * (T(1) / n2);
    }

    // Dot product
    T dot(const Quaternion &q) const
    {
        return w * q.w + x * q.x + y * q.y + z * q.z;
    }

    T angleBetween(const Quaternion<T> &q) const
    {
        T dotProduct = dot(q);
        // Clamp the dot product to the range [-1, 1] to avoid NaN from acos
        dotProduct = std::fmax(-1.0f, std::fmin(1.0f, dotProduct));
        return 2.0f * std::acos(dotProduct); // returns radians
    }

    T angleBetweenDeg(const Quaternion<T> &q) const
    {
        return angleBetween(q) * (T(180.0) / T(M_PI)); // Convert radians to degrees
    }

    // Quaternion multiplication
    Quaternion operator*(const Quaternion &q) const
    {
        return Quaternion(
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w,
            w * q.w - x * q.x - y * q.y - z * q.z);
    }

    // Scalar multiplication
    Quaternion operator*(T scalar) const
    {
        return Quaternion(x * scalar, y * scalar, z * scalar, w * scalar);
    }

    // Rotate a 3D vector
    Vector3<T> rotate(const Vector3<T> &v) const
    {
        Quaternion p(v.x, v.y, v.z, T(0));
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
        if (s < T(1e-6))                                 // Avoid division by zero
            return Vector3<T>(T(1), T(0), T(0)) * angle; // Default axis if no rotation
        return Vector3<T>(x / s, y / s, z / s) * angle;  // Axis * angle
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
    Vector3<T> yawPitchRoll(const Vector3<T> &g) const
    {
        T pitch = std::atan2(-g.x, std::sqrt(g.y * g.y + g.z * g.z)) * (T(180.0) / T(M_PI));
        T roll = std::atan2(g.y, g.z) * (T(180.0) / T(M_PI));

        // Extract yaw from quaternion
        T siny_cosp = T(2.0) * (w * z + x * y);
        T cosy_cosp = T(1.0) - T(2.0) * (y * y + z * z);
        T yaw = std::atan2(siny_cosp, cosy_cosp) * (T(180.0) / T(M_PI));

        return {yaw, pitch, roll};
    }

    Vector3<T> toEuler() const
    {
        T sinr_cosp = T(2.0) * (w * x + y * z);
        T cosr_cosp = T(1.0) - T(2.0) * (x * x + y * y);
        T roll = std::atan2(sinr_cosp, cosr_cosp);

        T sinp = T(2.0) * (w * y - z * x);
        if (std::abs(sinp) >= T(1.0))
            return {T(0), std::copysign(T(M_PI / 2), sinp), T(0)}; // Use 90 degrees if out of range
        T pitch = std::asin(sinp);

        T siny_cosp = T(2.0) * (w * z + x * y);
        T cosy_cosp = T(1.0) - T(2.0) * (y * y + z * z);
        T yaw = std::atan2(siny_cosp, cosy_cosp);

        return {yaw, pitch, roll};
    }

    // Estimate a reference vector in the ground plane (perpendicular to gravity) using the reference quaternion
    Vector3<T> referenceInPlane(const Vector3<T> &gravity, T tolerance = DEFAULT_TOLERANCE) const
    {
        // Define forward vector in local space
        Vector3<T> forward_local = {T(1), T(0), T(0)};

        // Rotate forward vector by reference quaternion
        Quaternion<T> f = {forward_local.x, forward_local.y, forward_local.z, T(0)};
        Quaternion<T> q_conj = {T(-x), T(-y), T(-z), T(w)};

        Quaternion<T> rotated = (*this) * f * q_conj;

        // Subtract gravity component to project into plane
        T g_mag = std::sqrt(gravity.x * gravity.x + gravity.y * gravity.y + gravity.z * gravity.z);
        if (g_mag < tolerance)
            return {T(0), T(0), T(0)};
        Vector3<T> g_unit = {gravity.x / g_mag, gravity.y / g_mag, gravity.z / g_mag};

        T dot = rotated.x * g_unit.x + rotated.y * g_unit.y + rotated.z * g_unit.z;
        Vector3<T> proj = {
            rotated.x - dot * g_unit.x,
            rotated.y - dot * g_unit.y,
            rotated.z - dot * g_unit.z};

        T proj_mag = std::sqrt(proj.x * proj.x + proj.y * proj.y + proj.z * proj.z);
        if (proj_mag < tolerance)
            return {T(0), T(0), T(0)};

        return {proj.x / proj_mag, proj.y / proj_mag, proj.z / proj_mag};
    }

    Quaternion<T> relative(const Quaternion<T> &q2)
    {
        Quaternion<T> q1_inv = {T(-x), T(-y), T(-z), T(w)}; // Conjugate (assuming unit quaternion)
        Quaternion<T> q_rel;
        q_rel.w = q1_inv.w * q2.w - q1_inv.x * q2.x - q1_inv.y * q2.y - q1_inv.z * q2.z;
        q_rel.x = q1_inv.w * q2.x + q1_inv.x * q2.w + q1_inv.y * q2.z - q1_inv.z * q2.y;
        q_rel.y = q1_inv.w * q2.y - q1_inv.x * q2.z + q1_inv.y * q2.w + q1_inv.z * q2.x;
        q_rel.z = q1_inv.w * q2.z + q1_inv.x * q2.y - q1_inv.y * q2.x + q1_inv.z * q2.w;
        q_rel.normalize();
        return q_rel;
    }

    enum class GravityMotionDirection : int8_t
    {
        Against = -1,
        Perpendicular = 0,
        With = 1,
    };
    GravityMotionDirection directionRelativeToGravity(const Vector3<T> &gravity_world, const T epsilon = T(1e-4))
    {
        // Forward in local frame
        Vector3<T> forward_local = {1.0f, 0.0f, 0.0f};

        // Rotate to world frame
        Vector3<T> forward_world = this->rotate(forward_local);

        T dot = forward_world.normalized().dot(gravity_world.normalized());

        // The dot product indicates the alignment of the forward direction with gravity:
        // - If dot > epsilon: "With" means forward is aligned with gravity.
        // - If dot < -epsilon: "Against" means forward is opposite to gravity.
        // - Otherwise: "Perpendicular" means forward is orthogonal to gravity.
        if (dot > epsilon)
            return GravityMotionDirection::With;
        else if (dot < -epsilon)
            return GravityMotionDirection::Against;
        else
            return GravityMotionDirection::Perpendicular;
    }

    T signedProjectedRotationDeg(const Quaternion<T> &q2, const Vector3<T> &gravity, const Vector3<T> &reference_in_plane, T tolerance = DEFAULT_TOLERANCE)
    {
        Quaternion<T> q_rel = relative(q2);
        T angle_rad = T(2.0) * std::atan2(
                                   std::sqrt(q_rel.x * q_rel.x + q_rel.y * q_rel.y + q_rel.z * q_rel.z),
                                   q_rel.w);

        T norm = q_rel.magnitude();
        if (norm < tolerance)
            return T(0);

        Vector3<T> rot_axis = {q_rel.x / norm, q_rel.y / norm, q_rel.z / norm};

        // Normalize gravity vector
        T g_mag = std::sqrt(gravity.x * gravity.x + gravity.y * gravity.y + gravity.z * gravity.z);
        if (g_mag < tolerance)
            return T(0);

        Vector3<T> g_unit = {gravity.x / g_mag, gravity.y / g_mag, gravity.z / g_mag};

        // Project rotation axis onto plane perpendicular to gravity
        T dot_g = rot_axis.x * g_unit.x + rot_axis.y * g_unit.y + rot_axis.z * g_unit.z;
        Vector3<T> proj = {
            rot_axis.x - dot_g * g_unit.x,
            rot_axis.y - dot_g * g_unit.y,
            rot_axis.z - dot_g * g_unit.z};

        T proj_mag = std::sqrt(proj.x * proj.x + proj.y * proj.y + proj.z * proj.z);
        if (proj_mag < tolerance)
            return T(0);

        // Normalize projected vector
        proj.x /= proj_mag;
        proj.y /= proj_mag;
        proj.z /= proj_mag;

        // Compute signed angle using cross product and dot product with plane normal (gravity direction)
        Vector3<T> ref = reference_in_plane; // Assumed to be normalized and in the ground plane
        // Cross product of ref and proj
        Vector3<T> cross = {
            ref.y * proj.z - ref.z * proj.y,
            ref.z * proj.x - ref.x * proj.z,
            ref.x * proj.y - ref.y * proj.x};
        // Dot product with plane normal (gravity direction)
        T sign = (cross.x * g_unit.x + cross.y * g_unit.y + cross.z * g_unit.z) >= T(0) ? T(1) : T(-1);

        return sign * angle_rad * proj_mag * (T(180.0) / T(M_PI));
    }
    static Quaternion fromAxisAngle(const Vector3<T> &axis, T angleRad)
    {
        T halfAngle = angleRad * T(0.5);
        T s = std::sin(halfAngle);
        return Quaternion(axis.x * s, axis.y * s, axis.z * s, std::cos(halfAngle));
    }
    // Static identity quaternion.
    // Convention: Quaternion(x, y, z, w) where w is the scalar component (last parameter).
    static Quaternion identity()
    {
        return Quaternion(T(0), T(0), T(0), T(1));
    }

    static T accumulateContinuousRotation(T &accumulated, T previous, T current, T maxAngle = T(180), T deadzone = T(0.001))
    {
        T delta = current - previous;

        T wrapRange = maxAngle * 2;

        while (delta > maxAngle)
            delta -= wrapRange;
        while (delta < -maxAngle)
            delta += wrapRange;

        if (std::abs(delta) > deadzone)
            accumulated += delta;

        return accumulated;
    }

    /**
     * @brief Tolerance value for floating-point comparisons in quaternion operations.
     *
     * This value is used to avoid errors due to floating-point precision issues,
     * such as division by zero or comparing near-zero magnitudes.
     *
     * Recommended value: T(1e-4) (can be adjusted as needed for your application).
     */
    static constexpr T DEFAULT_TOLERANCE = T(1e-4);
};

template<typename T>
class QuaternionRotationAccumulator {
public:
    Quaternion<T> q_start;

    QuaternionRotationAccumulator() : q_start(Quaternion<T>::identity()) {}

    // Constructor: set the starting orientation
    QuaternionRotationAccumulator(const Quaternion<T>& start)
        : q_start(start.normalized()) {}

    // Call regularly to get total rotation from start
    Vector3<T> getEulerFromStart(const Quaternion<T>& q_current) const {
        Quaternion<T> q_delta = q_start.inverse() * q_current;
        return q_delta.toEuler();  // Returns yaw, pitch, roll delta from start
    }

    Vector3<T> getAxisAngleFromStart(const Quaternion<T>& q_current) const {
        Quaternion<T> q_delta = q_start.inverse() * q_current;
        return q_delta.toAxisAngle();  // axis * angle in radians
    }

    T getTotalRotationAngle(const Quaternion<T>& q_current) const {
        Quaternion<T> q_delta = q_start.inverse() * q_current;
        Vector3<T> axisAngle = q_delta.toAxisAngle();
        return axisAngle.magnitude();  // returns total angle in radians
    }
};
