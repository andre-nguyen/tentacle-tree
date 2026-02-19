#include "alloc_helpers.h"
#include "point.h"
#include <algorithm>
#include <array>
#include <boost/pool/pool_alloc.hpp>
#include <cstddef>
#include <doctest/doctest.h>
#include <tt/tentacle_tree.hpp>
#include <vector>

using PointF = Point<float>;

TEST_CASE("allocator_standard_vs_pool") {
    constexpr std::size_t kBucketSize = 32;
    constexpr float kMinExtent = 1.0f;
    constexpr std::size_t kNumPoints = 1000;

    auto points = alloc_helpers::generateRandomPoints<PointF>(kNumPoints);

    SUBCASE("construction") {
        using StandardTree = alloc_helpers::StandardTree<PointF>;
        using FastPoolTree = alloc_helpers::FastPoolTree<PointF>;
        using PoolTree = alloc_helpers::PoolTree<PointF>;

        StandardTree standard_tree(kBucketSize, kMinExtent);
        FastPoolTree fast_pool_tree(kBucketSize, kMinExtent);
        PoolTree pool_tree(kBucketSize, kMinExtent);

        CHECK(standard_tree.root() == nullptr);
        CHECK(fast_pool_tree.root() == nullptr);
        CHECK(pool_tree.root() == nullptr);
    }

    SUBCASE("insert_points") {
        using StandardTree = alloc_helpers::StandardTree<PointF>;
        using FastPoolTree = alloc_helpers::FastPoolTree<PointF>;
        using PoolTree = alloc_helpers::PoolTree<PointF>;

        StandardTree standard_tree(kBucketSize, kMinExtent);
        FastPoolTree fast_pool_tree(kBucketSize, kMinExtent);
        PoolTree pool_tree(kBucketSize, kMinExtent);

        standard_tree.insert(points.begin(), points.end());
        fast_pool_tree.insert(points.begin(), points.end());
        pool_tree.insert(points.begin(), points.end());

        auto countPointsIterative = [](const auto *root) -> std::size_t {
            if (!root)
                return 0;
            std::size_t count = 0;
            std::vector<decltype(root)> nodes_to_visit;
            nodes_to_visit.push_back(root);
            while (!nodes_to_visit.empty()) {
                auto *node = nodes_to_visit.back();
                nodes_to_visit.pop_back();
                count += node->points.size();
                for (const auto &child : node->children) {
                    if (child) {
                        nodes_to_visit.push_back(child.get());
                    }
                }
            }
            return count;
        };

        CHECK(countPointsIterative(standard_tree.root()) == kNumPoints);
        CHECK(countPointsIterative(fast_pool_tree.root()) == kNumPoints);
        CHECK(countPointsIterative(pool_tree.root()) == kNumPoints);
    }

    SUBCASE("knn_search") {
        using StandardTree = alloc_helpers::StandardTree<PointF>;
        using FastPoolTree = alloc_helpers::FastPoolTree<PointF>;
        using PoolTree = alloc_helpers::PoolTree<PointF>;

        StandardTree standard_tree(kBucketSize, kMinExtent);
        FastPoolTree fast_pool_tree(kBucketSize, kMinExtent);
        PoolTree pool_tree(kBucketSize, kMinExtent);

        standard_tree.insert(points.begin(), points.end());
        fast_pool_tree.insert(points.begin(), points.end());
        pool_tree.insert(points.begin(), points.end());

        PointF query{0.0f, 0.0f, 0.0f};
        std::size_t k = 10;

        auto standard_results = standard_tree.knnSearch(query, k);
        auto fast_pool_results = fast_pool_tree.knnSearch(query, k);
        auto pool_results = pool_tree.knnSearch(query, k);

        CHECK(standard_results.size() == k);
        CHECK(fast_pool_results.size() == k);
        CHECK(pool_results.size() == k);

        auto getPointCoords = [](const auto &result) -> std::array<float, 3> {
            const auto &p = result.point.get();
            return std::array<float, 3>{p.x(), p.y(), p.z()};
        };

        for (std::size_t i = 0; i < k; ++i) {
            auto standard_coords = getPointCoords(standard_results[i]);
            auto fast_pool_coords = getPointCoords(fast_pool_results[i]);
            auto pool_coords = getPointCoords(pool_results[i]);

            for (int j = 0; j < 3; ++j) {
                CHECK(standard_coords[j] == doctest::Approx(fast_pool_coords[j]));
                CHECK(standard_coords[j] == doctest::Approx(pool_coords[j]));
            }
        }
    }

    SUBCASE("radius_search") {
        using StandardTree = alloc_helpers::StandardTree<PointF>;
        using FastPoolTree = alloc_helpers::FastPoolTree<PointF>;
        using PoolTree = alloc_helpers::PoolTree<PointF>;

        StandardTree standard_tree(kBucketSize, kMinExtent);
        FastPoolTree fast_pool_tree(kBucketSize, kMinExtent);
        PoolTree pool_tree(kBucketSize, kMinExtent);

        standard_tree.insert(points.begin(), points.end());
        fast_pool_tree.insert(points.begin(), points.end());
        pool_tree.insert(points.begin(), points.end());

        PointF query{0.0f, 0.0f, 0.0f};
        float radius = 50.0f;

        auto standard_results = standard_tree.radiusSearch(query, radius);
        auto fast_pool_results = fast_pool_tree.radiusSearch(query, radius);
        auto pool_results = pool_tree.radiusSearch(query, radius);

        CHECK(standard_results.size() == fast_pool_results.size());
        CHECK(fast_pool_results.size() == pool_results.size());

        auto getPointCoords = [](const std::reference_wrapper<const PointF> &ref) {
            const auto &p = ref.get();
            return std::array<float, 3>{p.x(), p.y(), p.z()};
        };

        std::vector<std::array<float, 3>> standard_coords;
        std::vector<std::array<float, 3>> fast_pool_coords;
        std::vector<std::array<float, 3>> pool_coords;

        for (const auto &r : standard_results) {
            standard_coords.push_back(getPointCoords(r));
        }
        for (const auto &r : fast_pool_results) {
            fast_pool_coords.push_back(getPointCoords(r));
        }
        for (const auto &r : pool_results) {
            pool_coords.push_back(getPointCoords(r));
        }

        std::sort(standard_coords.begin(), standard_coords.end());
        std::sort(fast_pool_coords.begin(), fast_pool_coords.end());
        std::sort(pool_coords.begin(), pool_coords.end());

        CHECK(standard_coords == fast_pool_coords);
        CHECK(fast_pool_coords == pool_coords);
    }

    SUBCASE("box_delete") {
        using StandardTree = alloc_helpers::StandardTree<PointF>;
        using FastPoolTree = alloc_helpers::FastPoolTree<PointF>;
        using PoolTree = alloc_helpers::PoolTree<PointF>;

        StandardTree standard_tree(kBucketSize, kMinExtent);
        FastPoolTree fast_pool_tree(kBucketSize, kMinExtent);
        PoolTree pool_tree(kBucketSize, kMinExtent);

        standard_tree.insert(points.begin(), points.end());
        fast_pool_tree.insert(points.begin(), points.end());
        pool_tree.insert(points.begin(), points.end());

        tt::BoundingBox<float> bbox{{-50.0f, -50.0f, -50.0f}, {50.0f, 50.0f, 50.0f}};

        standard_tree.boxDelete(bbox);
        fast_pool_tree.boxDelete(bbox);
        pool_tree.boxDelete(bbox);

        auto countPointsIterative = [](const auto *root) -> std::size_t {
            if (!root)
                return 0;
            std::size_t count = 0;
            std::vector<decltype(root)> nodes_to_visit;
            nodes_to_visit.push_back(root);
            while (!nodes_to_visit.empty()) {
                auto *node = nodes_to_visit.back();
                nodes_to_visit.pop_back();
                count += node->points.size();
                for (const auto &child : node->children) {
                    if (child) {
                        nodes_to_visit.push_back(child.get());
                    }
                }
            }
            return count;
        };

        CHECK(countPointsIterative(standard_tree.root()) ==
              countPointsIterative(fast_pool_tree.root()));
        CHECK(countPointsIterative(fast_pool_tree.root()) ==
              countPointsIterative(pool_tree.root()));
    }
}

TEST_CASE("allocator_type_traits") {
    using FastPoolTree = alloc_helpers::FastPoolTree<PointF>;
    using PoolTree = alloc_helpers::PoolTree<PointF>;
    using StandardTree = alloc_helpers::StandardTree<PointF>;

    static_assert(std::is_same_v<typename StandardTree::allocator_type, std::allocator<PointF>>);
    static_assert(
        std::is_same_v<typename FastPoolTree::allocator_type, boost::fast_pool_allocator<PointF>>);
    static_assert(std::is_same_v<typename PoolTree::allocator_type, boost::pool_allocator<PointF>>);

    constexpr std::size_t kBucketSize = 32;
    constexpr float kMinExtent = 1.0f;

    StandardTree standard_tree(kBucketSize, kMinExtent);
    FastPoolTree fast_pool_tree(kBucketSize, kMinExtent);
    PoolTree pool_tree(kBucketSize, kMinExtent);

    CHECK(standard_tree.allocator() == standard_tree.allocator());
    CHECK(fast_pool_tree.allocator() == fast_pool_tree.allocator());
    CHECK(pool_tree.allocator() == pool_tree.allocator());
}
