#pragma once

// This include is just to help auto
#include "tt/tentacle_tree.hpp"

#include <array>
#include <cstdint>

namespace tt {
namespace impl {

template <typename It>
concept ForwardPoint3dIterator = std::forward_iterator<It> && Point3d<std::iter_value_t<It>>;

/**
 *  Compute the axis-aligned bounding box for a set of 3D points.
 */
template <typename CoordT, ForwardPoint3dIterator BeginIt, ForwardPoint3dIterator EndIt>
auto computeBoundingBox(BeginIt begin, EndIt end) {
    std::array<CoordT, 3> min_coords = {std::numeric_limits<CoordT>::max(),
                                        std::numeric_limits<CoordT>::max(),
                                        std::numeric_limits<CoordT>::max()};
    std::array<CoordT, 3> max_coords = {std::numeric_limits<CoordT>::lowest(),
                                        std::numeric_limits<CoordT>::lowest(),
                                        std::numeric_limits<CoordT>::lowest()};
    for (auto it = begin; it != end; ++it) {
        const auto &p = *it;
        min_coords[0] = std::min(min_coords[0], p.x());
        min_coords[1] = std::min(min_coords[1], p.y());
        min_coords[2] = std::min(min_coords[2], p.z());
        max_coords[0] = std::max(max_coords[0], p.x());
        max_coords[1] = std::max(max_coords[1], p.y());
        max_coords[2] = std::max(max_coords[2], p.z());
    }
    return std::make_pair(min_coords, max_coords);
}

template <typename CoordT>
auto computeCenterAndMaxExtent(const std::array<CoordT, 3> &min_coords,
                               const std::array<CoordT, 3> &max_coords) {
    std::array<CoordT, 3> center;
    CoordT max_extent = std::numeric_limits<CoordT>::lowest();
    for (std::size_t i = 0; i < 3; ++i) {
        center[i] = min_coords[i];
        CoordT extent = (max_coords[i] - min_coords[i]) * CoordT(0.5);
        center[i] += extent;
        max_extent = std::max(max_extent, extent);
    }
    return std::make_pair(center, max_extent);
}

template <Point3d PointT, typename CoordT>
std::size_t computeMortonCode(const PointT &point, const std::array<CoordT, 3> &center) {
    // The code here only has to map to 8 octants, so its actually just 3 bits
    // I'm assuming the code was of type uint32_t so it could be used as an index directly
    // https://github.com/jbehley/octree/blob/8e3927d48d5ce61f94aab6090d77b4434f87dc89/Octree.hpp#L542C16-L542C26
    std::size_t code = 0;
    if (point.x() > center[0]) {
        code |= 0b0001;
    }
    if (point.y() > center[1]) {
        code |= 0b0010;
    }
    if (point.z() > center[2]) {
        code |= 0b0100;
    }
    return code;
}

} // namespace impl

template <Point3d PointT, typename CoordT>
TentacleTree<PointT, CoordT>::TentacleTree(std::size_t bucket_size, CoordT min_extent)
    : bucket_size_(bucket_size), min_extent_(min_extent) {}

template <Point3d PointT, typename CoordT>
template <std::random_access_iterator BeginIt, std::random_access_iterator EndIt>
void TentacleTree<PointT, CoordT>::insert(BeginIt begin, EndIt end) {
    // Section II.A:  Data Structure and Construction
    //     As for building an incremental octree, we firstly eliminate
    // invalid points, calculate the axis-aligned bounding box of all
    // valid points, and keep only indices and coordinates of valid
    // points. Then, starting at the root, the i-Octree recursively
    // splits the axis-aligned bounding box at the center into eight
    // cubes indexed by Morton codes and subdivides all the
    // points in current octant into each cube according to their cube
    // indices calculated. When a stopping criteria is satisfied, a leaf
    // octant will be created and a segment of continuous memory
    // will be allocated to store information in points of the leaf
    // node.

    // Note: It seems a lot of this code is actually the original octree paper code

    // Assume all points are valid and just find the bounding box
    auto [min_coords, max_coords] = impl::computeBoundingBox<CoordT>(begin, end);
    auto [center, max_extent] = impl::computeCenterAndMaxExtent<CoordT>(min_coords, max_coords);
    createOctant(center, max_extent, begin, end);
}

template <Point3d PointT, typename CoordT>
template <std::random_access_iterator BeginIt, std::random_access_iterator EndIt>
void TentacleTree<PointT, CoordT>::createOctant(std::array<CoordT, 3> center, CoordT half_extent,
                                                BeginIt begin, EndIt end) {
    // See algorithm 2 in J. Behley, V. Steinhage, A.B. Cremers. Efficient Radius Neighbor Search in
    // Three-dimensional Point Clouds, Proc. of the IEEE International Conference on Robotics and
    // Automation (ICRA), 2015.
    // Only difference is we get two iterators instead of indices
    root_ = Node<PointT, CoordT>{center, half_extent};
    const std::size_t num_points = std::distance(begin, end);
    bool keep_splitting = num_points > bucket_size_ && half_extent > 2 * min_extent_;
    if (keep_splitting) {
        // Take all the points and distribute them into octants according to their morton code
    } else {
        // This is a leaf node we can store the points now
        std::array<std::vector<PointT>, 8> octant_points;
    }
}

} // namespace tt