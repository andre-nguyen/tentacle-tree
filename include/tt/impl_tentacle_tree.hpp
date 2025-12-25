#pragma once

// This include is just to help auto
#include "tt/tentacle_tree.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>

namespace tt {
namespace impl {

template <typename It>
concept ForwardPoint3dIterator = std::forward_iterator<It> && Point3d<std::iter_value_t<It>>;

template <typename CoordT>
struct BoundingBox {
    std::array<CoordT, 3> min_coords;
    std::array<CoordT, 3> max_coords;
};

template <Point3d PointT>
bool isBoxInNode(const BoundingBox<typename Node<PointT>::CoordT> &box, const Node<PointT> &node) {
    using CoordT = typename Node<PointT>::CoordT;
    for (std::size_t i = 0; i < 3; ++i) {
        CoordT node_min = node.center[i] - node.half_extent;
        CoordT node_max = node.center[i] + node.half_extent;
        if (box.max_coords[i] < node_min || box.min_coords[i] > node_max) {
            return false;
        }
    }
    return true;
}

/**
 *  Compute the axis-aligned bounding box for a set of 3D points.
 */
template <ForwardPoint3dIterator BeginIt, ForwardPoint3dIterator EndIt>
auto computeBoundingBox(BeginIt begin, EndIt end) {
    using PointT = std::iter_value_t<BeginIt>;
    using CoordT = PointCoordinateTypeT<PointT>;

    BoundingBox<CoordT> bbox;
    bbox.min_coords = {std::numeric_limits<CoordT>::max(), std::numeric_limits<CoordT>::max(),
                       std::numeric_limits<CoordT>::max()};
    bbox.max_coords = {std::numeric_limits<CoordT>::lowest(), std::numeric_limits<CoordT>::lowest(),
                       std::numeric_limits<CoordT>::lowest()};
    for (auto it = begin; it != end; ++it) {
        const auto &p = *it;
        bbox.min_coords[0] = std::min(bbox.min_coords[0], p.x());
        bbox.min_coords[1] = std::min(bbox.min_coords[1], p.y());
        bbox.min_coords[2] = std::min(bbox.min_coords[2], p.z());
        bbox.max_coords[0] = std::max(bbox.max_coords[0], p.x());
        bbox.max_coords[1] = std::max(bbox.max_coords[1], p.y());
        bbox.max_coords[2] = std::max(bbox.max_coords[2], p.z());
    }
    return bbox;
}

template <typename CoordT>
auto computeCenterAndMaxExtent(const BoundingBox<CoordT> &bbox) {
    // Compute center and maximum half-extent from a BoundingBox
    std::array<CoordT, 3> center;
    CoordT max_extent = std::numeric_limits<CoordT>::lowest();
    for (std::size_t i = 0; i < 3; ++i) {
        center[i] = bbox.min_coords[i];
        CoordT extent = (bbox.max_coords[i] - bbox.min_coords[i]) * CoordT(0.5);
        center[i] += extent;
        max_extent = std::max(max_extent, extent);
    }
    return std::make_pair(center, max_extent);
}

template <Point3d PointT>
std::size_t computeMortonCode(const PointT &point,
                              const std::array<PointCoordinateTypeT<PointT>, 3> &center) {
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

template <Point3d PointT, ForwardPoint3dIterator BeginIt, ForwardPoint3dIterator EndIt>
std::array<std::vector<PointT>, 8>
distributePointsToOctants(BeginIt begin, EndIt end,
                          const std::array<PointCoordinateTypeT<PointT>, 3> &center) {
    std::array<std::vector<PointT>, 8> octant_points;
    for (auto it = begin; it != end; ++it) {
        const auto &p = *it;
        std::size_t morton_code = computeMortonCode<PointT>(p, center);
        octant_points[morton_code].push_back(p);
    }
    return octant_points;
}

template <typename CoordT>
std::array<CoordT, 3> computeOctantCenter(const std::array<CoordT, 3> &parent_center,
                                          CoordT parent_half_extent, std::size_t octant_idx) {
    std::array<CoordT, 3> child_center = parent_center;
    CoordT child_half_extent = parent_half_extent * CoordT(0.5);
    child_center[0] += ((octant_idx & 0b0001) ? child_half_extent : -child_half_extent);
    child_center[1] += ((octant_idx & 0b0010) ? child_half_extent : -child_half_extent);
    child_center[2] += ((octant_idx & 0b0100) ? child_half_extent : -child_half_extent);
    return child_center;
}

template <Point3d PointT>
std::array<PointCoordinateTypeT<PointT>, 3>
computeParentCenter(const Node<PointT> &node,
                    const std::array<PointCoordinateTypeT<PointT>, 3> &target) {
    using CoordT = PointCoordinateTypeT<PointT>;
    auto parent_extent = node.half_extent * CoordT(2.0);
    std::array<CoordT, 3> parent_center;
    // Move the root "up" or "down" in each axis based on which side the node is on
    for (std::size_t i = 0; i < 3; ++i) {
        if (node.center[i] >= target[i]) {
            parent_center[i] = node.center[i] + parent_extent;
        } else {
            parent_center[i] = node.center[i] - parent_extent;
        }
    }

    return parent_center;
}

/**
 * Creates a new parent node double the size of the input node, and links the input node as its
 * child.
 * @tparam PointT Point type used by the node. Must satisfy the Point3d concept.
 * @param node The node which will be wrapped by a new parent node.
 * @param bbox The bounding box towards which we are expanding until we encompass it.
 * @return The new parent node, with the input node linked as its child.
 */
template <Point3d PointT>
std::unique_ptr<Node<PointT>>
makeParentAndLinkNode(std::unique_ptr<Node<PointT>> node,
                      const BoundingBox<PointCoordinateTypeT<PointT>> &bbox) {
    using CoordT = Node<PointT>::CoordT;

    auto new_half_extent = node->half_extent * CoordT(2);
    std::array<CoordT, 3> new_center = node->center;

    // Grow towards the max of the bbox

    // Grow towards the min of the bbox
}

} // namespace impl

template <Point3d PointT>
TentacleTree<PointT>::TentacleTree(std::size_t bucket_size, CoordT min_extent)
    : bucket_size_(bucket_size), min_extent_(min_extent) {}

template <Point3d PointT>
template <std::random_access_iterator BeginIt, std::random_access_iterator EndIt>
void TentacleTree<PointT>::insert(BeginIt begin, EndIt end) {
    if (!root_) {
        init(begin, end);
        return;
    }

    // Section II.B:  Dynamic Updates
    //    1) Incremental Update: When inserting new points, we
    // have to consider the situation that some points may be
    // beyond the boundary of the axis-aligned bounding box of
    // the original tree. Once there are points out of the range of
    // octree, we have to expand the bounding box by creating new
    // root octant whose children contain current root octant. This
    // process may be executed several times to ensure that all new
    // points are within the range of the tree. Then, new points are
    // added to the expanded octree (see Fig. 3).

    auto bbox = impl::computeBoundingBox(begin, end);
    bool needs_expansion = !impl::isBoxInNode<PointT>(bbox, *root_);
}

template <Point3d PointT>
template <std::random_access_iterator BeginIt, std::random_access_iterator EndIt>
void TentacleTree<PointT>::init(BeginIt begin, EndIt end) {
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

    // Assume all points are valid and just find the bounding box
    auto bbox = impl::computeBoundingBox(begin, end);
    auto [center, max_extent] = impl::computeCenterAndMaxExtent(bbox);
    root_ = createOctant(center, max_extent, begin, end);
}

template <Point3d PointT>
template <std::random_access_iterator BeginIt, std::random_access_iterator EndIt>
std::unique_ptr<Node<PointT>>
TentacleTree<PointT>::createOctant(const std::array<CoordT, 3> &center, CoordT half_extent,
                                   BeginIt begin, EndIt end) {
    // See algorithm 2 in J. Behley, V. Steinhage, A.B. Cremers. Efficient Radius Neighbor Search in
    // Three-dimensional Point Clouds, Proc. of the IEEE International Conference on Robotics and
    // Automation (ICRA), 2015.
    // Only difference is we get two iterators instead of indices
    auto octant = std::make_unique<Node<PointT>>(center, half_extent);
    const auto num_points = static_cast<std::size_t>(std::distance(begin, end));
    bool keep_splitting = num_points > bucket_size_ && half_extent > 2 * min_extent_;
    if (keep_splitting) {
        auto octant_points = impl::distributePointsToOctants<PointT>(begin, end, center);
        CoordT child_half_extent = half_extent * CoordT(0.5);
        for (std::size_t octant_idx = 0; octant_idx < 8; ++octant_idx) {
            auto &points_in_octant = octant_points[octant_idx];
            if (points_in_octant.empty()) {
                continue;
            }

            const auto child_center = impl::computeOctantCenter(center, half_extent, octant_idx);
            octant->children[octant_idx] = createOctant(
                child_center, child_half_extent, points_in_octant.begin(), points_in_octant.end());
        }
    } else {
        // This is a leaf node we can store the points now
        octant->points.reserve(num_points);
        std::copy(begin, end, std::back_inserter(octant->points));
    }
    return octant;
}

} // namespace tt
