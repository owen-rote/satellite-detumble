#pragma once

#include <array>

#include "raylib.h"

namespace satellite_detumble {
void DrawHud(Vector3 angularVelocity, Vector3 angularVelocityEstimate);

// Draws a scrolling graph of true (red) and filtered (blue) angular velocity magnitude.
void DrawVelocityGraph(const std::array<float, 240>& history, const std::array<float, 240>& estimateHistory,
                       int nextIndex, float maxSpeed);

}  // namespace satellite_detumble
