#ifndef TT_TENTACLE_TREE_HPP
#define TT_TENTACLE_TREE_HPP

#include <concepts>
#include <cstdint>
#include <memory>
#include <vector>

namespace tt {

using Array3f = std::array<float, 3>;

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

template <typename T>
concept Array3 = requires(T p) {
    { p[0] } -> std::convertible_to<typename T::value_type>;
    { p[1] } -> std::convertible_to<typename T::value_type>;
    { p[2] } -> std::convertible_to<typename T::value_type>;
};

template <typename CoordT>
struct BoundingBox {
    std::array<CoordT, 3> min_coords;
    std::array<CoordT, 3> max_coords;
};

// Type trait to extract the coordinate type
template <typename T>
struct PointCoordinateType {
    using Type = typename T::value_type;
};

// Helper alias for convenience
template <typename T>
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
struct KnnResult {
    using CoordT = PointCoordinateTypeT<PointT>;
    std::reference_wrapper<const PointT> point;
    CoordT distance;
};

template <Point3d PointT>
bool operator<(const KnnResult<PointT> &a, const KnnResult<PointT> &b) {
    return a.distance < b.distance;
}

template <Point3d PointT>
class TentacleTree {
  public:
    using CoordT = PointCoordinateTypeT<PointT>;

    TentacleTree(std::size_t bucket_size, CoordT min_extent);

    template <std::random_access_iterator BeginIt, std::random_access_iterator EndIt>
    void insert(BeginIt begin, EndIt end);

    void boxDelete(const BoundingBox<CoordT> &box);

    using SearchResult = std::vector<KnnResult<PointT>>;
    SearchResult knnSearch(const PointT &query_point, std::size_t k);

    /**
     * Perform a radius search where any point within @param radius will be returned.
     * @param query_point Center of the search ball
     * @param radius Radius of the search ball
     * @return References to all points found. Invalid if a call to insert is done.
     */
    std::vector<std::reference_wrapper<const PointT>> radiusSearch(const PointT &query_point,
                                                                   CoordT radius);

    Node<PointT> *root() const { return root_.get(); }

  private:
    std::size_t bucket_size_;
    CoordT min_extent_; // aka e_min

    std::unique_ptr<Node<PointT>> root_;

    template <std::random_access_iterator BeginIt, std::random_access_iterator EndIt>
    void init(BeginIt begin, EndIt end);

    template <std::random_access_iterator BeginIt, std::random_access_iterator EndIt>
    std::unique_ptr<Node<PointT>> createNode(const std::array<CoordT, 3> &center,
                                             CoordT half_extent, BeginIt begin, EndIt end);

    std::unique_ptr<Node<PointT>> boxDelete(const BoundingBox<CoordT> &box,
                                            std::unique_ptr<Node<PointT>> node);
};

} // namespace tt

#include "impl_tentacle_tree.hpp"

#endif // TT_TENTACLE_TREE_HPP