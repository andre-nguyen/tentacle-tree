#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace tt {



template <typename ScalarT>
struct PointXYZRGB {
    using Scalar = ScalarT;
    using value_type = ScalarT; // compatibility with existing Point concept

    // Public storage (aggregate-friendly)
    Scalar coords[3] = {Scalar(0), Scalar(0), Scalar(0)};
    std::uint8_t color[3] = {0, 0, 0};

    // Defaulted special members
    PointXYZRGB() = default;
    PointXYZRGB(const PointXYZRGB &) = default;
    PointXYZRGB &operator=(const PointXYZRGB &) = default;

    // Construct from coordinates and optional color
    PointXYZRGB(Scalar x, Scalar y, Scalar z, std::uint8_t r = 0, std::uint8_t g = 0,
                std::uint8_t b = 0) {
        coords[0] = x;
        coords[1] = y;
        coords[2] = z;
        color[0] = r;
        color[1] = g;
        color[2] = b;
    }

    // Aggregate-style constructor from std::array for convenience
    PointXYZRGB(const std::array<Scalar, 3> &c,
                const std::array<std::uint8_t, 3> &col = {0, 0, 0}) {
        coords[0] = c[0];
        coords[1] = c[1];
        coords[2] = c[2];
        color[0] = col[0];
        color[1] = col[1];
        color[2] = col[2];
    }

    // Coordinate accessors
    Scalar x() const { return coords[0]; }
    Scalar y() const { return coords[1]; }
    Scalar z() const { return coords[2]; }
    Scalar &x() { return coords[0]; }
    Scalar &y() { return coords[1]; }
    Scalar &z() { return coords[2]; }

    Scalar &operator[](std::size_t idx) { return coords[idx]; }
    const Scalar &operator[](std::size_t idx) const { return coords[idx]; }

    // Color accessors
    std::uint8_t r() const { return color[0]; }
    std::uint8_t g() const { return color[1]; }
    std::uint8_t b() const { return color[2]; }

    std::uint8_t &r() { return color[0]; }
    std::uint8_t &g() { return color[1]; }
    std::uint8_t &b() { return color[2]; }

    // Allow structured bindings with coords and color by providing getters
    std::array<Scalar, 3> toArray() const { return {coords[0], coords[1], coords[2]}; }
    std::array<std::uint8_t, 3> toColorArray() const { return {color[0], color[1], color[2]}; }
};

} // namespace tt
