#pragma once

#include <array>

#include "raylib.h"

namespace rotation {
void DrawHud(Vector3 angularVelocity);

void DrawVelocityGraph(const std::array<float, 240>& history, int nextIndex, float maxSpeed);

}  // namespace rotation
