#ifndef TT_TENTACLE_TREE_HPP
#define TT_TENTACLE_TREE_HPP

#include <concepts>
#include <cstdint>
#include <memory>
#include <vector>

namespace tt {

template <typename T>
concept Point3d = requires(T p) {
    // Ensure we can access x, y, z as methods and operator[]
    { p.x() } -> std::convertible_to<typename T::Float>;
    { p.y() } -> std::convertible_to<typename T::Float>;
    { p.z() } -> std::convertible_to<typename T::Float>;
    { p[0] } -> std::convertible_to<typename T::Float>;
    { p[1] } -> std::convertible_to<typename T::Float>;
    { p[2] } -> std::convertible_to<typename T::Float>;
};

// Type trait to extract the coordinate type from a Point3d conforming type
template <Point3d T>
struct PointCoordinateType {
    using Type = typename T::Float;
};

// Helper alias for convenience
template <Point3d T>
using PointCoordinateTypeT = typename PointCoordinateType<T>::Type;

template <Point3d PointT>
struct Node {
    using CoordT = PointCoordinateTypeT<PointT>;

    std::array<CoordT, 3> center; // aka c_o
    CoordT half_extent;           // aka e_o

    // Note both vectors are actually contrained on the max size, so we could feasibly use
    // boost::static_vector
    std::array<std::unique_ptr<Node>, 8>
        children;               // Paper says pointer to fist child, but vector is easier to manage
    std::vector<PointT> points; // ?? Paper mentions coordinates AND indices?

    Node() : center{CoordT(0), CoordT(0), CoordT(0)}, half_extent(CoordT(0)) {}

    Node(const std::array<CoordT, 3> &center_, CoordT half_extent_)
        : center(center_), half_extent(half_extent_) {}
};

template <Point3d PointT>
class TentacleTree {
  public:
    using CoordT = PointCoordinateTypeT<PointT>;

    TentacleTree(std::size_t bucket_size, CoordT min_extent);

    template <std::random_access_iterator BeginIt, std::random_access_iterator EndIt>
    void insert(BeginIt begin, EndIt end);

    Node<PointT> *root() const { return root_.get(); }

  private:
    std::size_t bucket_size_;
    CoordT min_extent_; // aka e_min

    std::unique_ptr<Node<PointT>> root_;

    template <std::random_access_iterator BeginIt, std::random_access_iterator EndIt>
    void init(BeginIt begin, EndIt end);

    template <std::random_access_iterator BeginIt, std::random_access_iterator EndIt>
    std::unique_ptr<Node<PointT>> createOctant(const std::array<CoordT, 3> &center,
                                               CoordT half_extent, BeginIt begin, EndIt end);
};

} // namespace tt

#include "impl_tentacle_tree.hpp"

#endif // TT_TENTACLE_TREE_HPP