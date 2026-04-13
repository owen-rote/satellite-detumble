# Satellite De-Tumble Demo

Real-time 3D visualization of a PD attitude controller driving a tumbling satellite to a fixed target orientation. C++17 + [raylib](https://www.raylib.com/). 

Press `R` to randomize.

![demo](demo.gif)

## Control loop (per fixed step)

1. Multiply target quaternion by the inverse of the current one to get the attitude error
2. Convert that error quaternion to an axis-angle vector. Magnitude is how far off, direction is which way to rotate
3. PD torque command: scale the error vector by Kp, subtract angular rate scaled by Kd
4. Clamp torque magnitude to the actuator limit
5. Add torque to angular velocity
6. Rotate the current quaternion by the angular velocity delta, renormalize

## Code structure

- `src/main.cpp` - Render loop, no math
- `src/simulation.cpp` - dynamics, PD controller, quaternion integrator
- `src/ui.cpp` - HUD
- `include/rotation/simulation.hpp` - simulation state and dynamics API
- `include/rotation/ui.hpp` - rendering API

## Build

```bash
# Linux
cmake -S . -B build && cmake --build build  # (also fetches raylib)
./build/rotation
```
