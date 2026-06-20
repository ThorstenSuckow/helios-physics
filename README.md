# helios::physics

Physics integration module for the helios C++23 ecosystem.

## Overview

`helios::physics` currently provides motion integration primitives built on top of
`helios::engine`, `helios::ecs`, and `helios::math`.

## Features

- Physics module aggregation via `helios.physics`
- Motion namespace split (`components`, `systems`)
- Velocity component alias for 3D local-space motion
- Delta-time based position integration system

## Module surface

| Area | Public modules / APIs |
|------|------------------------|
| Aggregator | `helios.physics` |
| Motion | `helios.physics.motion` |
| Components | `helios.physics.motion.components`, `Velocity3DComponent` |
| Systems | `helios.physics.motion.systems`, `MotionIntegrationSystem` |

## Usage

### C++ module

```cpp
import helios.physics;
```

### CMake

When used as a subdirectory, link against:

```cmake
add_subdirectory(path/to/helios-physics)
target_link_libraries(your_target PRIVATE helios::physics)
```

## Development

Build and run local tests:

```bash
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

## Related repositories

- [`helios-engine`](https://github.com/thorstensuckow/helios-engine)
- [`helios-ecs`](https://github.com/thorstensuckow/helios-ecs)
- [`helios-math`](https://github.com/thorstensuckow/helios-math)
- [`helios-opengl`](https://github.com/thorstensuckow/helios-opengl)

