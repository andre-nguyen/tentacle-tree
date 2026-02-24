# Tentacle-tree

Basically a reimplementation of [i-Octree: A Fast, Lightweight, and Dynamic Octree for Proximity Search](https://arxiv.org/abs/2309.08315) for fun and education.

J. Zhu, H. Li, Z. Wang, S. Wang and T. Zhang, "i-Octree: A Fast, Lightweight, and Dynamic Octree for Proximity Search," 2024 IEEE International Conference on Robotics and Automation (ICRA), Yokohama, Japan, 2024, pp. 12290-12296, doi: 10.1109/ICRA57147.2024.10611019.

## Building

1. Clone [vcpkg](https://github.com/microsoft/vcpkg).
2. Create a local `CMakeUserPresets.json` file with a `configurePreset` that sets the `VCPKG_ROOT` environment variable.
```json
{
    "version": 3,
    "cmakeMinimumRequired": {
        "major": 3,
        "minor": 28,
        "patch": 0
    },
    "configurePresets": [
        {
            "name": "release",
            "displayName": "Release",
            "inherits": ["release-base"],
            "environment": {
                "VCPKG_ROOT": "/some/path/here"
            }
        }
    ],
    "buildPresets": [
        {
            "name": "release",
            "configurePreset": "release"
        }
    ],
    "testPresets": [
        {
            "name": "release",
            "configurePreset": "release",
            "output": {
                "outputOnFailure": true
            }
        }
    ]
}
```
3. Configure & Build
```bash
cmake --preset release
cmake --build --preset release
```
4. Run unit tests
```bash
ctest --preset release
```
