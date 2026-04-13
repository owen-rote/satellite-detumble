#pragma once

#include <array>

#include "raylib.h"
#include "raymath.h"

namespace rotation {
constexpr int kGraphSamples = 240;

struct SimulationState {
    Quaternion orientation = QuaternionIdentity();
    Vector3 angularVelocity = Vector3Zero();
    std::array<float, kGraphSamples> angularVelocityHistory = {};
    int historyIndex = 0;
    float accumulator = 0.0f;
};

void ResetSimulation(SimulationState& state);

void UpdateSimulation(SimulationState& state, Quaternion targetOrientation, float frameTime);

}  // namespace rotation
