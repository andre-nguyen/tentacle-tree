#ifndef TESTS_RERUN_H
#define TESTS_RERUN_H

#include "point.h"
#include "tt/tentacle_tree.hpp"
#include <numeric>
#include <optional>
#include <vector>

// Provide real rerun declarations when available, otherwise provide minimal stubs so code
// depending on `rerun.h` compiles even when the SDK is not present.
#ifdef RERUN_ENABLED
#include <rerun.hpp>
#else
namespace rerun {
struct Position3D {
    double x, y, z;
    Position3D(double a, double b, double c) : x(a), y(b), z(c) {}
};

namespace components {
struct Translation3D {
    float x, y, z;
    Translation3D(float a, float b, float c) : x(a), y(b), z(c) {}
};
struct HalfSize3D {
    float x, y, z;
    HalfSize3D(float a, float b, float c) : x(a), y(b), z(c) {}
};
} // namespace components

struct Color {
    int r, g, b;
    Color(int rr, int gg, int bb) : r(rr), g(gg), b(bb) {}
};

struct Boxes3D {
    static Boxes3D from_centers_and_half_sizes(const std::vector<components::Translation3D> &,
                                               const std::vector<components::HalfSize3D> &) {
        return Boxes3D();
    }
    static Boxes3D from_mins_and_sizes(const std::vector<std::array<float, 3>> &,
                                       const std::vector<std::array<float, 3>> &) {
        return Boxes3D();
    }
    Boxes3D with_colors(const Color &) const { return *this; }
};
} // namespace rerun
#endif

template <typename FloatT>
std::vector<rerun::Position3D> toRerunPositions(const std::vector<Point<FloatT>> &points) {
    std::vector<rerun::Position3D> rr_positions;
    rr_positions.reserve(points.size());
    for (const auto &p : points) {
        rr_positions.emplace_back(static_cast<double>(p.x()), static_cast<double>(p.y()),
                                  static_cast<double>(p.z()));
    }
    return rr_positions;
}

template <typename FloatT>
std::vector<rerun::Position3D> toRerunPositions(const tt::TentacleTree<Point<FloatT>> &tree) {
    std::vector<rerun::Position3D> rr_positions;
    const auto root = tree.root();
    const auto leaves = tt::impl::collectLeafNodes(*root);
    auto total = std::accumulate(leaves.cbegin(), leaves.cend(), std::size_t(0),
                                 [](std::size_t acc, const tt::Node<Point<FloatT>> *node) {
                                     return acc + node->points.size();
                                 });
    rr_positions.reserve(total);
    for (const auto *leaf : leaves) {
        for (const auto &p : leaf->points) {
            rr_positions.emplace_back(static_cast<double>(p.x()), static_cast<double>(p.y()),
                                      static_cast<double>(p.z()));
        }
    }
    return rr_positions;
}

template <tt::Point3d PointT>
void toRerunBoxes(const tt::Node<PointT> &node,
                  std::vector<rerun::components::Translation3D> &centers,
                  std::vector<rerun::components::HalfSize3D> &half_sizes,
                  const std::optional<rerun::Color> &color = std::nullopt) {
    centers.emplace_back(static_cast<float>(node.center[0]), static_cast<float>(node.center[1]),
                         static_cast<float>(node.center[2]));
    half_sizes.emplace_back(static_cast<float>(node.half_extent),
                            static_cast<float>(node.half_extent),
                            static_cast<float>(node.half_extent));
    for (const auto &child : node.children) {
        if (!child) {
            continue;
        }
        toRerunBoxes(*child, centers, half_sizes, color);
    }
}

template <tt::Point3d PointT>
rerun::Boxes3D toRerunBoxes(const tt::Node<PointT> &node,
                            const std::optional<rerun::Color> &color = std::nullopt) {
    std::vector<rerun::components::Translation3D> centers;
    std::vector<rerun::components::HalfSize3D> half_sizes;
    toRerunBoxes(node, centers, half_sizes, color);
    if (color.has_value()) {
        return rerun::Boxes3D::from_centers_and_half_sizes(centers, half_sizes).with_colors(*color);
    }
    return rerun::Boxes3D::from_centers_and_half_sizes(centers, half_sizes);
}

template <tt::Point3d PointT>
rerun::Boxes3D toRerunBoxes(const tt::TentacleTree<PointT> &tree,
                            const std::optional<rerun::Color> &color = std::nullopt) {
    const auto root = tree.root();
    return toRerunBoxes(*root, color);
}

template <typename FloatT>
rerun::Boxes3D toRerunBoxes(const tt::BoundingBox<FloatT> &bbox,
                            const std::optional<rerun::Color> &color = std::nullopt) {
    if (color.has_value()) {
        return rerun::Boxes3D::from_mins_and_sizes(
                   {{bbox.min_coords[0], bbox.min_coords[1], bbox.min_coords[2]}},
                   {{bbox.max_coords[0] - bbox.min_coords[0],
                     bbox.max_coords[1] - bbox.min_coords[1],
                     bbox.max_coords[2] - bbox.min_coords[2]}})
            .with_colors(*color);
    }
    return rerun::Boxes3D::from_mins_and_sizes(
        {{bbox.min_coords[0], bbox.min_coords[1], bbox.min_coords[2]}},
        {{bbox.max_coords[0] - bbox.min_coords[0], bbox.max_coords[1] - bbox.min_coords[1],
          bbox.max_coords[2] - bbox.min_coords[2]}});
}

#endif // TESTS_RERUN_H
