#pragma once

// This include is just to help autocomplete
#include "tt/tentacle_tree.hpp"

#include "tt/impl/bounded_priority_queue.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>

namespace tt {
namespace impl {

template <typename It>
concept ForwardPoint3dIterator = std::forward_iterator<It> && Point3d<std::iter_value_t<It>>;

template <Point3d PointT>
bool isPointInNode(const std::array<typename Node<PointT>::CoordT, 3> &point,
                   const Node<PointT> &node) {
    const auto &center = node.center;
    const auto &half_extent = node.half_extent;
    // !! allow boundary points to be inside the node
    return point[0] <= center[0] + half_extent && point[0] >= center[0] - half_extent &&
           point[1] <= center[1] + half_extent && point[1] >= center[1] - half_extent &&
           point[2] <= center[2] + half_extent && point[2] >= center[2] - half_extent;
}

template <Point3d PointT>
bool isBoxInNode(const BoundingBox<typename Node<PointT>::CoordT> &box, const Node<PointT> &node) {
    return isPointInNode<PointT>(box.min_coords, node) &&
           isPointInNode<PointT>(box.max_coords, node);
}

template <Point3d PointT>
auto toBoundingBox(const Node<PointT> &node) {
    using CoordT = PointCoordinateTypeT<PointT>;
    BoundingBox<CoordT> bbox;
    bbox.min_coords = {node.center[0] - node.half_extent, node.center[1] - node.half_extent,
                       node.center[2] - node.half_extent};
    bbox.max_coords = {node.center[0] + node.half_extent, node.center[1] + node.half_extent,
                       node.center[2] + node.half_extent};
    return bbox;
}

template <Point3d PointT>
bool isBoxOverlappingNode(const BoundingBox<typename Node<PointT>::CoordT> &a,
                          const Node<PointT> &node) {
    auto b = toBoundingBox<PointT>(node);
    return (a.min_coords[0] <= b.max_coords[0] && a.max_coords[0] >= b.min_coords[0]) &&
           (a.min_coords[1] <= b.max_coords[1] && a.max_coords[1] >= b.min_coords[1]) &&
           (a.min_coords[2] <= b.max_coords[2] && a.max_coords[2] >= b.min_coords[2]);
}

template <Point3d PointT>
bool isNodeInBox(const Node<PointT> &node, const BoundingBox<typename Node<PointT>::CoordT> &box) {
    // A node is fully inside a box if all corners of the node are inside the box
    // We can check this by verifying the node's bounding box min/max are inside the target box
    return box.min_coords[0] <= node.center[0] - node.half_extent &&
           box.min_coords[1] <= node.center[1] - node.half_extent &&
           box.min_coords[2] <= node.center[2] - node.half_extent &&
           box.max_coords[0] >= node.center[0] + node.half_extent &&
           box.max_coords[1] >= node.center[1] + node.half_extent &&
           box.max_coords[2] >= node.center[2] + node.half_extent;
}

template <Point3d PointT>
bool isSphereInNode(const Node<PointT> &node, const PointT &sphere_center,
                    typename Node<PointT>::CoordT sphere_radius) {
    // A sphere is fully inside a node if the sphere doesn't extend beyond any of the node's faces
    // For each axis, check: sphere_center - radius >= node_min AND sphere_center + radius <=
    // node_max
    using CoordT = typename Node<PointT>::CoordT;

    for (std::size_t i = 0; i < 3; ++i) {
        CoordT node_min = node.center[i] - node.half_extent;
        CoordT node_max = node.center[i] + node.half_extent;
        CoordT sphere_min = sphere_center[i] - sphere_radius;
        CoordT sphere_max = sphere_center[i] + sphere_radius;

        // If the sphere extends beyond either boundary, it's not fully inside
        if (sphere_min < node_min || sphere_max > node_max) {
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

template <Array3 PointT>
std::size_t computeMortonCode(const PointT &point,
                              const std::array<PointCoordinateTypeT<PointT>, 3> &center) {
    // The code here only has to map to 8 octants, so its actually just 3 bits
    // I'm assuming the code was of type uint32_t so it could be used as an index directly
    // https://github.com/jbehley/octree/blob/8e3927d48d5ce61f94aab6090d77b4434f87dc89/Octree.hpp#L542C16-L542C26
    std::size_t code = 0;
    if (point[0] > center[0]) {
        code |= 0b0001;
    }
    if (point[1] > center[1]) {
        code |= 0b0010;
    }
    if (point[2] > center[2]) {
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
    // auto parent_extent = node.half_extent * CoordT(2.0);
    std::array<CoordT, 3> parent_center;
    // Move the root "towards" the target in each axis
    for (std::size_t i = 0; i < 3; ++i) {
        if (node.center[i] < target[i]) {
            parent_center[i] = node.center[i] + node.half_extent;
        } else {
            parent_center[i] = node.center[i] - node.half_extent;
        }
    }

    return parent_center;
}

/**
 * Creates a new parent node double the size of the input node, and links the input node as its
 * child.
 * @tparam PointT Point type used by the node. Must satisfy the Point3d concept.
 * @param node The node which will be wrapped by a new parent node.
 * @param target The point towards which we wish to grow
 * @return The new parent node, with the input node linked as its child.
 */
template <Point3d PointT>
std::unique_ptr<Node<PointT>>
makeParentAndLinkNode(std::unique_ptr<Node<PointT>> node,
                      const std::array<PointCoordinateTypeT<PointT>, 3> &target) {
    using CoordT = PointCoordinateTypeT<PointT>;
    auto parent_node = std::make_unique<Node<PointT>>();
    parent_node->half_extent = node->half_extent * CoordT(2.0);
    parent_node->center = computeParentCenter(*node, target);

    auto morton_code = computeMortonCode(node->center, parent_node->center);
    parent_node->children[morton_code] = std::move(node);

    return parent_node;
}

/**
 * Spread tentacles (grows the tree) until the given bounding box is fully encompassed by the tree.
 * @tparam PointT Point type stored inside a node
 * @param node The root node from which we grow the tree
 * @param bbox The bounding box we are trying to grow around
 * @return New root node of the tree
 */
template <Point3d PointT>
std::unique_ptr<Node<PointT>>
growTreeUntilAbsorption(std::unique_ptr<Node<PointT>> node,
                        const BoundingBox<PointCoordinateTypeT<PointT>> &bbox) {
    std::unique_ptr<Node<PointT>> root = std::move(node);
    while (!isPointInNode(bbox.max_coords, *root)) {
        root = makeParentAndLinkNode(std::move(root), bbox.max_coords);
    }

    while (!isPointInNode(bbox.min_coords, *root)) {
        root = makeParentAndLinkNode(std::move(root), bbox.min_coords);
    }
    return root;
}

template <std::random_access_iterator BeginIt, std::random_access_iterator EndIt, Point3d PointT>
void insert(BeginIt begin, EndIt end, Node<PointT> &node, std::size_t bucket_size,
            PointCoordinateTypeT<PointT> min_extent);

template <std::random_access_iterator BeginIt, std::random_access_iterator EndIt, Point3d PointT>
void splitAndInsertPoints(BeginIt begin, EndIt end, Node<PointT> &node,
                          const std::size_t bucket_size,
                          const PointCoordinateTypeT<PointT> min_extent) {
    using CoordT = PointCoordinateTypeT<PointT>;
    auto octant_points = impl::distributePointsToOctants<PointT>(begin, end, node.center);
    CoordT child_half_extent = node.half_extent * CoordT(0.5);
    for (std::size_t octant_idx = 0; octant_idx < 8; ++octant_idx) {
        auto &points_in_octant = octant_points[octant_idx];
        if (points_in_octant.empty()) {
            continue;
        }

        const auto child_center =
            impl::computeOctantCenter(node.center, node.half_extent, octant_idx);
        if (!node.children[octant_idx]) {
            node.children[octant_idx] =
                std::make_unique<Node<PointT>>(child_center, child_half_extent);
        }
        insert(points_in_octant.begin(), points_in_octant.end(), *node.children[octant_idx],
               bucket_size, min_extent);
    }
}

template <std::random_access_iterator BeginIt, std::random_access_iterator EndIt, Point3d PointT>
void insertIntoLeaf(BeginIt begin, EndIt end, Node<PointT> &node, const std::size_t bucket_size,
                    std::size_t num_points) {
    // TODO: downsampling
    // For now just add the points we can
    auto new_size = node.points.size() + num_points;
    if (new_size > bucket_size) {
        new_size = bucket_size;
    }
    node.points.reserve(new_size);
    for (auto it = begin; it != end && node.points.size() < new_size; ++it) {
        node.points.push_back(*it);
    }
}

template <Point3d PointT>
bool isNodeSmallestPossible(const Node<PointT> &node, PointCoordinateTypeT<PointT> min_extent) {
    return node.half_extent <= 2 * min_extent;
}

template <Point3d PointT>
bool hasChildren(const Node<PointT> &node) {
    return std::any_of(node.children.begin(), node.children.end(),
                       [](const std::unique_ptr<Node<PointT>> &c) { return c != nullptr; });
}

template <std::random_access_iterator BeginIt, std::random_access_iterator EndIt, Point3d PointT>
void insert(BeginIt begin, EndIt end, Node<PointT> &node, const std::size_t bucket_size,
            const PointCoordinateTypeT<PointT> min_extent) {
    //     The process of adding new points to an octant Co is similar
    // to the construction of an octant. If Co is a leaf node and it
    // satisfies the subdivision criteria, all points (i.e., old points and
    // new added points) in Co will be recursively subdivided into
    // child octants. If down-sampling is enabled, eo ≤ 2emin , and
    // |Po | > b/8, new points will be deleted later instead of being
    // added to Co .
    //     Otherwise, a segment of continuous memory
    // will be allocated for the updated points. If Co has child
    // octants, the problem becomes assigning the newly added
    // points to various children, and only the new points are to
    // be subdivided. This process is similar to the one mentioned
    // above, except for the recursive updating of octants
    const bool is_smallest_possible = isNodeSmallestPossible(node, min_extent);
    const auto num_points = std::distance(begin, end);
    const bool needs_splitting = num_points > bucket_size && !is_smallest_possible;
    if (needs_splitting) {
        splitAndInsertPoints(begin, end, node, bucket_size, min_extent);
    } else {
        insertIntoLeaf(begin, end, node, bucket_size, num_points);
    }
}

template <Point3d PointT>
void collectLeafNodes(std::vector<const Node<PointT> *> &leaves, const Node<PointT> &node) {
    bool is_leaf = true;
    for (const auto &child : node.children) {
        if (child != nullptr) {
            is_leaf = false;
            collectLeafNodes(leaves, *child);
        }
    }
    if (is_leaf) {
        leaves.push_back(&node);
    }
}

template <Point3d PointT>
std::vector<const Node<PointT> *> collectLeafNodes(const Node<PointT> &node) {
    std::vector<const Node<PointT> *> leaves;
    collectLeafNodes(leaves, node);
    return leaves;
}

template <Point3d PointT>
std::vector<std::vector<PointT> *> collectLeafPoints(std::vector<const Node<PointT> *> &leaves) {
    std::vector<std::vector<PointT> *> leaf_points;
    for (const auto &leaf : leaves) {
        leaf_points.push_back(&leaf->points);
    }
    return leaf_points;
}

template <Point3d PointT>
std::vector<std::vector<PointT> *> collectLeafPoints(const Node<PointT> &node) {
    auto leaves = collectLeafNodes<PointT>(node);
    return collectLeafPoints<PointT>(leaves);
}

template <Point3d PointT>
auto distance(const PointT &a, const PointT &b) {
    using CoordT = PointCoordinateTypeT<PointT>;
    CoordT dx = a[0] - b[0];
    CoordT dy = a[1] - b[1];
    CoordT dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

template <typename FloatT>
auto distance(const std::array<FloatT, 3> &a, const std::array<FloatT, 3> &b) {
    FloatT dx = a[0] - b[0];
    FloatT dy = a[1] - b[1];
    FloatT dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

template <Point3d PointT>
auto distance(const Node<PointT> &node, const PointT &point) {
    return distance(node.center, point);
}

template <Point3d PointT>
Node<PointT> &findClosestLeafNode(Node<PointT> &node, const PointT &point,
                                  PointCoordinateTypeT<PointT> min_extent) {
    // TODO: This might be wrong and should check if the children are nullptr instead
    if (isNodeSmallestPossible<PointT>(node, min_extent)) {
        return node;
    }

    const auto morton_code = impl::computeMortonCode(point, node.center);
    auto closest_child = node.children[morton_code].get();

    if (closest_child) {
        return findClosestLeafNode(*closest_child, point, min_extent);
    }

    // No children, return current node
    return node;
}

template <typename PointT>
bool knnSearch(const Node<PointT> &node, const PointT &query_point, std::size_t k,
               BoundedPriorityQueue<KnnResult<PointT>> &queue) {
    // II. C. Nearest Neighbor Search
    // I'm not sure I fully understand the paper's description of the algorithm here
    // Sounds like a depth first search of the tree following the morton code.
    // If after hitting the closest leaf we still need more neighbors, then we search the rest of
    // the nodes.
    // There is also an early exit clause with the search ball that allows us to not search other
    // nodes.
    if (!impl::hasChildren(node)) {
        for (const auto &point : node.points) {
            KnnResult<PointT> result{point, impl::distance<PointT>(query_point, point)};
            queue.push(result);
        }
        return queue.isFull() and impl::isSphereInNode(node, query_point, queue.top().distance);
    }

    auto morton_code = impl::computeMortonCode(query_point, node.center);
    bool stop_search = knnSearch(*node.children[morton_code], query_point, k, queue);
    if (stop_search) {
        return stop_search;
    }

    // TODO: Implement sorted lookup table instead of iterating blindly
    for (std::size_t i = 0; i < 8; i++) {
        if (i == morton_code) {
            continue;
        }
        if (node.children[i] == nullptr) {
            continue;
        }

        // TODO: early exit if node can't possibly contain something better
        stop_search = knnSearch(*node.children[i], query_point, k, queue);
        if (stop_search) {
            return stop_search;
        }
    }

    // Is it possible for other branches to contain something better?
    // Quote
    //   If h is full and the search ball S(q, dmax ) defined by
    //   q and the largest distance dmax in h is inside the axis-aligned
    //   box of current octant, the searching is over.
    return queue.isFull() and isSphereInNode(node, query_point, queue.top().distance);
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
    root_ = impl::growTreeUntilAbsorption(std::move(root_), bbox);
    impl::insert(begin, end, *root_, bucket_size_, min_extent_);
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
    root_ = createNode(center, max_extent, begin, end);
}

template <Point3d PointT>
template <std::random_access_iterator BeginIt, std::random_access_iterator EndIt>
std::unique_ptr<Node<PointT>> TentacleTree<PointT>::createNode(const std::array<CoordT, 3> &center,
                                                               CoordT half_extent, BeginIt begin,
                                                               EndIt end) {
    // See algorithm 2 in J. Behley, V. Steinhage, A.B. Cremers. Efficient Radius Neighbor Search in
    // Three-dimensional Point Clouds, Proc. of the IEEE International Conference on Robotics and
    // Automation (ICRA), 2015.
    // Only difference is we get two iterators instead of indices
    auto node = std::make_unique<Node<PointT>>(center, half_extent);
    const auto num_points = static_cast<std::size_t>(std::distance(begin, end));
    const bool is_node_smallest_possible = impl::isNodeSmallestPossible(*node, min_extent_);
    bool keep_splitting = num_points > bucket_size_ && !is_node_smallest_possible;
    if (keep_splitting) {
        auto octant_points = impl::distributePointsToOctants<PointT>(begin, end, center);
        CoordT child_half_extent = half_extent * CoordT(0.5);
        for (std::size_t octant_idx = 0; octant_idx < 8; ++octant_idx) {
            auto &points_in_octant = octant_points[octant_idx];
            if (points_in_octant.empty()) {
                continue;
            }

            const auto child_center = impl::computeOctantCenter(center, half_extent, octant_idx);
            node->children[octant_idx] = createNode(
                child_center, child_half_extent, points_in_octant.begin(), points_in_octant.end());
        }
    } else {
        // This is a leaf node we can store the points now
        node->points.reserve(num_points);
        std::copy(begin, end, std::back_inserter(node->points));
    }
    return node;
}

template <Point3d PointT>
void TentacleTree<PointT>::boxDelete(const BoundingBox<CoordT> &box) {
    root_ = boxDelete(box, std::move(root_));
}

template <Point3d PointT>
std::unique_ptr<Node<PointT>> TentacleTree<PointT>::boxDelete(const BoundingBox<CoordT> &box,
                                                              std::unique_ptr<Node<PointT>> node) {
    if (impl::isNodeInBox<PointT>(*node, box)) {
        return nullptr;
    }

    if (impl::isBoxOverlappingNode(box, *node)) {
        const bool is_leaf = impl::hasChildren(*node);
        if (is_leaf) {
            return nullptr;
        }

        for (auto &child : node->children) {
            if (child) {
                child = boxDelete(box, std::move(child));
            }
        }
    }

    return std::move(node);
}

template <Point3d PointT>
TentacleTree<PointT>::SearchResult TentacleTree<PointT>::knnSearch(const PointT &query_point,
                                                                   std::size_t k) {
    impl::BoundedPriorityQueue<KnnResult<PointT>> queue(std::less<KnnResult<PointT>>(), k);
    impl::knnSearch(*root_, query_point, k, queue);
    return queue.destructiveGet();
}

} // namespace tt
