#include "rotation/ui.hpp"

#include <algorithm>

#include "raymath.h"

namespace rotation {
void DrawHud(Vector3 angularVelocity) {
    // Status card
    DrawRectangle(16, 16, 470, 172, Fade(SKYBLUE, 0.18f));
    DrawText("Satellite De-Tumble Demo", 28, 28, 26, DARKBLUE);
    DrawText("Fixed-step update: 60 Hz", 28, 60, 20, DARKGRAY);
    DrawText("Controller: simple PD torque command", 28, 86, 20, DARKGRAY);
    DrawText(TextFormat("|w| = %.3f rad/s", Vector3Length(angularVelocity)), 28, 112, 20, MAROON);
    DrawText("Blue line = current forward, green = target", 28, 138, 18, DARKGRAY);
    DrawText("Press R to randomize and restart", 28, 160, 18, DARKGRAY);
}

void DrawVelocityGraph(const std::array<float, 240>& history, int nextIndex, float maxSpeed) {
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

    // history graph loop
    for (int sample = 1; sample < 240; ++sample) {
        const int previousIndex = (nextIndex + sample - 1) % 240;
        const int currentIndex = (nextIndex + sample) % 240;

        const float x0 = left + width * (static_cast<float>(sample - 1) / 239.0f);
        const float x1 = left + width * (static_cast<float>(sample) / 239.0f);
        const float y0 = bottom - (history[previousIndex] / clampedMax) * height;
        const float y1 = bottom - (history[currentIndex] / clampedMax) * height;

        DrawLineEx(Vector2{x0, y0}, Vector2{x1, y1}, 2.0f, RED);
    }

    DrawText(TextFormat("%.2f", clampedMax), static_cast<int>(left), static_cast<int>(top) - 18, 16, DARKGRAY);
    DrawText("0.00", static_cast<int>(left), static_cast<int>(bottom) - 8, 16, DARKGRAY);
}

}  // namespace rotation
