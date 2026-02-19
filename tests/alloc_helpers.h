#ifndef TESTS_ALLOC_HELPERS_H
#define TESTS_ALLOC_HELPERS_H

#include <random>
#include <vector>

#include <boost/pool/pool_alloc.hpp>
#include <tt/pool_allocator.hpp>
#include <tt/tentacle_tree.hpp>

namespace alloc_helpers {

// Type aliases for convenience
template <typename PointT>
using StandardTree = tt::TentacleTree<PointT, std::allocator<PointT>>;

template <typename PointT>
using FastPoolTree = tt::TentacleTree<PointT, boost::fast_pool_allocator<PointT>>;

template <typename PointT>
using PoolTree = tt::TentacleTree<PointT, boost::pool_allocator<PointT>>;

template <typename PointT>
using PreallocatedPoolTree = tt::TentacleTree<PointT, tt::preallocated_pool_allocator<PointT>>;

// Generate random points with fixed seed for reproducibility
template <typename PointT>
std::vector<PointT> generateRandomPoints(std::size_t n, float range = 100.0f,
                                         std::uint_fast32_t seed = 12345) {
    std::minstd_rand gen(seed);
    std::uniform_real_distribution<float> dist(-range, range);

    std::vector<PointT> points;
    points.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        points.emplace_back(static_cast<typename PointT::Float>(dist(gen)),
                            static_cast<typename PointT::Float>(dist(gen)),
                            static_cast<typename PointT::Float>(dist(gen)));
    }
    return points;
}

} // namespace alloc_helpers

#endif // TESTS_ALLOC_HELPERS_H
