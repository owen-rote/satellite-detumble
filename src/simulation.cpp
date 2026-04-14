#include "simulation.hpp"

#include <algorithm>
#include <cmath>

#include "raymath.h"

namespace satellite_detumble {
namespace {
// Deterministic cadence for stable controller behavior across frame rates
constexpr float kFixedDt = 1.0f / 60.0f;
// Clamp extreme frame spikes
constexpr float kMaxFrameTime = 0.25f;
// Internal control torque setting. Change for faster or slower de-tumbling
constexpr float kTorqueLimit = 1.5f;
// Principal moments of inertia (kg*m^2) per axis.
// Uneven to simulation an asymmetrical satellite
constexpr Vector3 kInertiaPrincipal = {1.0f, 2.5f, 0.8f};

float RandomFloat(float minValue, float maxValue) {
    const float randomUnit = static_cast<float>(GetRandomValue(0, 10000)) / 10000.0f;
    return minValue + (maxValue - minValue) * randomUnit;
}

Vector3 ClampMagnitude(Vector3 value, float maxLength) {
    const float length = Vector3Length(value);

    if (length > maxLength && length > 0.0f) {
        return Vector3Scale(value, maxLength / length);
    }

    return value;
}

Vector3 QuaternionErrorVector(Quaternion current, Quaternion target) {
    // Right-multiply by inverse(current) to get target-in-current attitude error.
    Quaternion error = QuaternionMultiply(target, QuaternionInvert(current));

    // Enforce shortest-arc representation (q and -q encode identical rotation).
    if (error.w < 0.0f) {
        error.x = -error.x;
        error.y = -error.y;
        error.z = -error.z;
        error.w = -error.w;
    }

    Vector3 axis = Vector3{1.0f, 0.0f, 0.0f};
    float angle = 0.0f;
    QuaternionToAxisAngle(error, &axis, &angle);

    if (angle > PI) {
        angle -= 2.0f * PI;
    }

    // Guard axis-angle noise near zero rotation to avoid normalization artifacts.
    if (std::fabs(angle) < 0.0001f || Vector3LengthSqr(axis) < 0.0001f) {
        return Vector3Zero();
    }

    return Vector3Scale(Vector3Normalize(axis), angle);
}

Quaternion IntegrateOrientation(Quaternion orientation, Vector3 angularVelocity, float dt) {
    const float speed = Vector3Length(angularVelocity);

    if (speed < 0.0001f) {
        return orientation;
    }

    // Increment attitude with exponential map, then renormalize to limit drift.
    const Vector3 axis = Vector3Scale(angularVelocity, 1.0f / speed);
    const Quaternion delta = QuaternionFromAxisAngle(axis, speed * dt);
    return QuaternionNormalize(QuaternionMultiply(delta, orientation));
}

}  // namespace

void ResetSimulation(SimulationState& state) {
    // Start from a broad random attitude
    state.orientation = QuaternionNormalize(QuaternionFromEuler(RandomFloat(-120.0f * DEG2RAD, 120.0f * DEG2RAD),
                                                                RandomFloat(-120.0f * DEG2RAD, 120.0f * DEG2RAD),
                                                                RandomFloat(-120.0f * DEG2RAD, 120.0f * DEG2RAD)));

    // Seed initial tumble rate
    state.angularVelocity = Vector3{RandomFloat(-3.0f, 3.0f), RandomFloat(-3.0f, 3.0f), RandomFloat(-3.0f, 3.0f)};

    state.angularVelocityHistory.fill(0.0f);
    state.historyIndex = 0;
    state.accumulator = 0.0f;
}

void UpdateSimulation(SimulationState& state, Quaternion targetOrientation, float frameTime) {
    state.accumulator += std::min(frameTime, kMaxFrameTime);

    while (state.accumulator >= kFixedDt) {
        const Vector3 attitudeError = QuaternionErrorVector(state.orientation, targetOrientation);

        // PD law: P term drives attitude error to zero, D term damps angular rate.
        const float proportionalGain = 10.0f;
        const float dampingGain = 5.5f;

        Vector3 commandedTorque = Vector3Add(Vector3Scale(attitudeError, proportionalGain),
                                             Vector3Scale(state.angularVelocity, -dampingGain));

        commandedTorque = ClampMagnitude(commandedTorque, kTorqueLimit);

        // Euler rotation: dw/dt = I^-1 * (T - w x (I*w))
        //
        // The w x (I*w) part is what makes rotation feel real—it links the axes together
        // Without it, each axis spins on its own and you lose wobble/precession.
        const Vector3 Iw = {kInertiaPrincipal.x * state.angularVelocity.x,
                            kInertiaPrincipal.y * state.angularVelocity.y,
                            kInertiaPrincipal.z * state.angularVelocity.z};
        const Vector3 netTorque = Vector3Subtract(commandedTorque, Vector3CrossProduct(state.angularVelocity, Iw));
        const Vector3 angularAccel = {netTorque.x / kInertiaPrincipal.x, netTorque.y / kInertiaPrincipal.y,
                                      netTorque.z / kInertiaPrincipal.z};
        state.angularVelocity = Vector3Add(state.angularVelocity, Vector3Scale(angularAccel, kFixedDt));
        state.orientation = IntegrateOrientation(state.orientation, state.angularVelocity, kFixedDt);

        state.angularVelocityHistory[state.historyIndex] = Vector3Length(state.angularVelocity);
        state.historyIndex = (state.historyIndex + 1) % kGraphSamples;
        state.accumulator -= kFixedDt;
    }
}

}  // namespace satellite_detumble
