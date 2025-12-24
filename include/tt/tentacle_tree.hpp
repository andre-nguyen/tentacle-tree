#pragma once

#include <concepts>
#include <cstdint>
#include <memory>
#include <vector>

namespace tt {

template <typename T>
concept Point3d = requires(T p) {
    // Ensure we can access x, y, z as methods and operator[]
    { p.x() } -> std::convertible_to<typename T::ValueT>;
    { p.y() } -> std::convertible_to<typename T::ValueT>;
    { p.z() } -> std::convertible_to<typename T::ValueT>;
    { p[0] } -> std::convertible_to<typename T::ValueT>;
    { p[1] } -> std::convertible_to<typename T::ValueT>;
    { p[2] } -> std::convertible_to<typename T::ValueT>;
};

template <Point3d PointT, typename CoordT>
struct Node {
    CoordT center[3]; // aka c_o
    CoordT half_extent;    // aka e_o

    // Note both vectors are actually contrained on the max size, so we could feasibly use
    // boost::static_vector
    std::vector<std::unique_ptr<Node>>
        children;               // Paper says pointer to fist child, but vector is easier to manage
    std::vector<PointT> points; // ?? Paper mentions coordinates AND indices?

    Node() {
        center[0] = CoordT(0);
        center[1] = CoordT(0);
        center[2] = CoordT(0);
        half_extent = CoordT(0);
    }
};

template <Point3d PointT, typename CoordT>
class TentacleTree {
  public:
    TentacleTree(std::size_t bucket_size, CoordT min_extent);

    template <std::random_access_iterator BeginIt, std::random_access_iterator EndIt>
    void insert(BeginIt begin, EndIt end);

  private:
    std::size_t bucket_size_;
    CoordT min_extent_; // aka e_min

    Node<PointT, CoordT> root_;

    template <std::random_access_iterator BeginIt, std::random_access_iterator EndIt>
    void createOctant(std::array<CoordT, 3> center, CoordT half_extent, BeginIt begin, EndIt end);
};

} // namespace tt

#include "impl_tentacle_tree.hpp"
