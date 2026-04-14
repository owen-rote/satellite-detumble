#pragma once

#include <array>

#include "raylib.h"
#include "raymath.h"

namespace satellite_detumble {
// Ring-buffer sample count used for the on-screen angular velocity trace.
constexpr int kGraphSamples = 240;

struct SimulationState {
    // Body attitude
    Quaternion orientation = QuaternionIdentity();
    // True body angular velocity in rad/s (ground truth, not available to the controller).
    Vector3 angularVelocity = Vector3Zero();
    // Low-pass filtered gyro estimate fed to the controller.
    Vector3 angularVelocityEstimate = Vector3Zero();
    // True and estimated magnitude histories for the HUD graph
    std::array<float, kGraphSamples> angularVelocityHistory = {};
    std::array<float, kGraphSamples> angularVelocityEstimateHistory = {};
    // Next write index into angularVelocityHistory.
    int historyIndex = 0;
    // Frame-time accumulator used to run a deterministic fixed-step update.
    float accumulator = 0.0f;
};

// Randomizes initial attitude/rate and clears graph/history state.
void ResetSimulation(SimulationState& state);

// Advances the simulation from variable frame time to fixed integration steps.
void UpdateSimulation(SimulationState& state, Quaternion targetOrientation, float frameTime);

}  // namespace satellite_detumble
