#include "ui.hpp"

#include <algorithm>

#include "raymath.h"

namespace satellite_detumble {
void DrawHud(Vector3 angularVelocity, Vector3 angularVelocityEstimate) {
    // Status card
    DrawRectangle(16, 16, 470, 196, Fade(SKYBLUE, 0.18f));
    DrawText("Satellite De-Tumble Demo", 28, 28, 26, DARKBLUE);
    DrawText("Fixed-step update: 60 Hz", 28, 60, 20, DARKGRAY);
    DrawText("Controller: PD with low-pass filtered gyro", 28, 86, 20, DARKGRAY);
    DrawText(TextFormat("|w| true     = %.3f rad/s", Vector3Length(angularVelocity)), 28, 112, 20, DARKGRAY);
    DrawText(TextFormat("|w| filtered = %.3f rad/s", Vector3Length(angularVelocityEstimate)), 28, 136, 20, MAROON);
    DrawText("Blue line = current forward, green = target", 28, 162, 18, DARKGRAY);
    DrawText("Press R to randomize", 28, 184, 18, DARKGRAY);
}

void DrawVelocityGraph(const std::array<float, 240>& history, const std::array<float, 240>& estimateHistory,
                       int nextIndex, float maxSpeed) {
    const Rectangle bounds = {860.0f, 24.0f, 390.0f, 180.0f};  // anchored to the upper-right corner.

    DrawRectangleRounded(bounds, 0.08f, 12, Fade(DARKBLUE, 0.12f));
    DrawRectangleLinesEx(bounds, 1.0f, Fade(DARKBLUE, 0.35f));
    DrawText("Angular Velocity", 878, 36, 22, DARKBLUE);
    DrawText("rad/s", 1138, 38, 18, DARKGRAY);

    const float left = bounds.x + 14.0f;
    const float right = bounds.x + bounds.width - 14.0f;
    const float bottom = bounds.y + bounds.height - 16.0f;
    const float top = bounds.y + 56.0f;
    const float height = bottom - top;
    const float width = right - left;

    // Cartesian plot frame and horizontal guide lines for quick trend reading
    DrawLine(static_cast<int>(left), static_cast<int>(bottom), static_cast<int>(right), static_cast<int>(bottom),
             Fade(BLACK, 0.35f));
    DrawLine(static_cast<int>(left), static_cast<int>(top), static_cast<int>(left), static_cast<int>(bottom),
             Fade(BLACK, 0.35f));

    for (int i = 1; i < 4; ++i) {
        const float y = top + (height / 4.0f) * static_cast<float>(i);
        DrawLine(static_cast<int>(left), static_cast<int>(y), static_cast<int>(right), static_cast<int>(y),
                 Fade(DARKGRAY, 0.15f));
    }

    // keep graph scale reasonable
    const float clampedMax = std::max(maxSpeed, 0.1f);

    // True rate (red) and filtered estimate (blue) drawn in separate passes so
    // the estimate is always visible on top.
    for (int sample = 1; sample < 240; ++sample) {
        const int prev = (nextIndex + sample - 1) % 240;
        const int curr = (nextIndex + sample) % 240;

        const float x0 = left + width * (static_cast<float>(sample - 1) / 239.0f);
        const float x1 = left + width * (static_cast<float>(sample) / 239.0f);

        const float y0 = bottom - (history[prev] / clampedMax) * height;
        const float y1 = bottom - (history[curr] / clampedMax) * height;
        DrawLineEx(Vector2{x0, y0}, Vector2{x1, y1}, 2.0f, Fade(RED, 0.5f));
    }

    for (int sample = 1; sample < 240; ++sample) {
        const int prev = (nextIndex + sample - 1) % 240;
        const int curr = (nextIndex + sample) % 240;

        const float x0 = left + width * (static_cast<float>(sample - 1) / 239.0f);
        const float x1 = left + width * (static_cast<float>(sample) / 239.0f);

        const float y0 = bottom - (estimateHistory[prev] / clampedMax) * height;
        const float y1 = bottom - (estimateHistory[curr] / clampedMax) * height;
        DrawLineEx(Vector2{x0, y0}, Vector2{x1, y1}, 2.0f, BLUE);
    }

    // Legend
    DrawRectangle(static_cast<int>(right) - 130, static_cast<int>(top) - 2, 12, 12, Fade(RED, 0.5f));
    DrawText("true", static_cast<int>(right) - 114, static_cast<int>(top) - 2, 14, DARKGRAY);
    DrawRectangle(static_cast<int>(right) - 70, static_cast<int>(top) - 2, 12, 12, BLUE);
    DrawText("filtered", static_cast<int>(right) - 54, static_cast<int>(top) - 2, 14, DARKGRAY);

    DrawText(TextFormat("%.2f", clampedMax), static_cast<int>(left), static_cast<int>(top) - 18, 16, DARKGRAY);
    DrawText("0.00", static_cast<int>(left), static_cast<int>(bottom) - 8, 16, DARKGRAY);
}

}  // namespace satellite_detumble
