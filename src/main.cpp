#include "raylib.h"
#include "raymath.h"
#include "simulation.hpp"
#include "ui.hpp"

namespace {
constexpr int kScreenWidth = 1280;
constexpr int kScreenHeight = 720;
}  // namespace

int main() {
    InitWindow(kScreenWidth, kScreenHeight, "rotation");
    SetTargetFPS(60);  // Lock to 60hz

    // init camera
    Camera3D camera = {};
    camera.position = Vector3{8.0f, 5.0f, 8.0f};
    camera.target = Vector3{0.0f, 1.0f, 0.0f};
    camera.up = Vector3{0.0f, 1.0f, 0.0f};
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    // init cube mesh
    Mesh cubeMesh = GenMeshCube(2.0f, 2.0f, 2.0f);
    Model cubeModel = LoadModelFromMesh(cubeMesh);
    cubeModel.materials[0].maps[MATERIAL_MAP_DIFFUSE].color = RED;

    satellite_detumble::SimulationState simulation = {};

    // init target state as position and quaternion
    const Quaternion targetOrientation = QuaternionFromEuler(0.0f, 35.0f * DEG2RAD, 0.0f);
    const Vector3 cubePosition = Vector3{0.0f, 1.0f, 0.0f};

    satellite_detumble::ResetSimulation(simulation);

    while (!WindowShouldClose()) {
        if (IsKeyPressed(KEY_R)) {
            // Upon R: another random Attitude and velocity
            satellite_detumble::ResetSimulation(simulation);
        }

        satellite_detumble::UpdateSimulation(simulation, targetOrientation, GetFrameTime());

        // Convert quaternion to axis-angle because DrawModelEx expects that form.
        Vector3 drawAxis = Vector3{0.0f, 1.0f, 0.0f};
        float drawAngle = 0.0f;
        QuaternionToAxisAngle(simulation.orientation, &drawAxis, &drawAngle);

        // Visual cues: green = target direction, blue = current body-forward.
        const Vector3 targetForward = Vector3RotateByQuaternion(Vector3{0.0f, 0.0f, 2.8f}, targetOrientation);
        const Vector3 currentForward = Vector3RotateByQuaternion(Vector3{0.0f, 0.0f, 2.4f}, simulation.orientation);

        BeginDrawing();
        ClearBackground(RAYWHITE);

        BeginMode3D(camera);
        DrawPlane(Vector3{0.0f, 0.0f, 0.0f}, Vector2{16.0f, 16.0f}, LIGHTGRAY);
        DrawGrid(16, 1.0f);
        DrawModelEx(cubeModel, cubePosition, drawAxis, drawAngle * RAD2DEG, Vector3One(), RED);
        DrawModelWiresEx(cubeModel, cubePosition, drawAxis, drawAngle * RAD2DEG, Vector3One(), MAROON);
        DrawLine3D(cubePosition, Vector3Add(cubePosition, targetForward), GREEN);
        DrawLine3D(cubePosition, Vector3Add(cubePosition, currentForward), BLUE);
        DrawSphere(Vector3Add(cubePosition, targetForward), 0.12f, GREEN);
        EndMode3D();

        // draw info overlays
        satellite_detumble::DrawHud(simulation.angularVelocity);
        satellite_detumble::DrawVelocityGraph(simulation.angularVelocityHistory, simulation.historyIndex, 4.0f);

        EndDrawing();
    }

    // cleanup
    UnloadModel(cubeModel);
    CloseWindow();
    return 0;
}