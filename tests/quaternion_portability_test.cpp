#include <cmath>
#include <cstdint>
#include <numbers>

#include "Quaternion.hpp"

namespace {
template <typename T>
bool approximatelyEqual(T actual, T expected, T tolerance = T(1e-5))
{
    return std::abs(actual - expected) <= tolerance;
}
}

int main()
{
    const Quaternion<float> identity;
    const Quaternion<float> quarterTurn = Quaternion<float>::fromAxisAngle(
        0.0f, 0.0f, 1.0f, std::numbers::pi_v<float> / 2.0f);
    const Quaternion<float> halfTurn = Quaternion<float>::fromAxisAngle(
        0.0f, 0.0f, 1.0f, std::numbers::pi_v<float>);

    if (!approximatelyEqual(identity.angleBetweenDeg(identity), 0.0f) ||
        !approximatelyEqual(identity.angleBetweenDeg(quarterTurn), 90.0f) ||
        !approximatelyEqual(identity.angleBetweenDeg(halfTurn), 180.0f))
        return 1;

    const Vector3<float> yawPitchRoll = quarterTurn.yawPitchRoll(Vector3<float>(0.0f, 0.0f, 1.0f));
    if (!approximatelyEqual(yawPitchRoll.x, 90.0f) ||
        !approximatelyEqual(yawPitchRoll.y, 0.0f) ||
        !approximatelyEqual(yawPitchRoll.z, 0.0f))
        return 1;

    const Quaternion<float> pitchBoundary = Quaternion<float>::fromAxisAngle(
        0.0f, 1.0f, 0.0f, std::numbers::pi_v<float> / 2.0f);
    const Vector3<float> euler = pitchBoundary.toEuler();
    return approximatelyEqual(euler.y, std::numbers::pi_v<float> / 2.0f) ? 0 : 1;
}
