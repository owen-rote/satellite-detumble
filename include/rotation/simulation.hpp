#pragma once

#include <array>

#include "raylib.h"
#include "raymath.h"

namespace rotation {
// Ring-buffer sample count used for the on-screen angular velocity trace.
constexpr int kGraphSamples = 240;

struct SimulationState {
    // Body attitude
    Quaternion orientation = QuaternionIdentity();
    // Body angular velocity vector in rad/s.
    Vector3 angularVelocity = Vector3Zero();
    // Magnitude history used by the HUD graph (circular buffer).
    std::array<float, kGraphSamples> angularVelocityHistory = {};
    // Next write index into angularVelocityHistory.
    int historyIndex = 0;
    // Frame-time accumulator used to run a deterministic fixed-step update.
    float accumulator = 0.0f;
};

// Randomizes initial attitude/rate and clears graph/history state.
void ResetSimulation(SimulationState& state);

// Advances the simulation from variable frame time to fixed integration steps.
void UpdateSimulation(SimulationState& state, Quaternion targetOrientation, float frameTime);

}  // namespace rotation
