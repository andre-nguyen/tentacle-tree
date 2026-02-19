#include "alloc_helpers.h"
#include "point.h"
#include "tt/impl/asc_io.hpp"
#include "tt/tentacle_tree.hpp"
#include <algorithm>
#include <benchmark/benchmark.h>
#include <ranges>
#include <vector>

namespace {

std::vector<Point<float>> loadMatterhorn() {
    auto cloud = tt::loadASC<float>("data/matterhorn.asc");
    std::vector<Point<float>> points;
    points.reserve(cloud.coords.size());
    std::ranges::transform(cloud.coords, std::back_inserter(points),
                           [](const std::array<float, 3> &point) {
                               return Point<float>{point[0], point[1], point[2]};
                           });
    return points;
}

template <class TreeT>
static void BM_insert(benchmark::State &state) {
    using PointT = Point<float>;
    const auto bucket_size = state.range(0);

    auto points = loadMatterhorn();

    for (auto _ : state) {
        TreeT tree(bucket_size, 1.0f);
        tree.insert(points.begin(), points.end());
    }
}

template <class TreeT>
static void BM_knn(benchmark::State &state) {
    using PointT = Point<float>;
    const auto bucket_size = state.range(0);

    auto points = loadMatterhorn();
    TreeT tree(bucket_size, 1.0f);
    tree.insert(points.begin(), points.end());

    const PointT query{0.0f, 0.0f, 0.0f};
    const std::size_t k = 10;

    for (auto _ : state) {
        auto results = tree.knnSearch(query, k);
        benchmark::DoNotOptimize(results);
    }
}

template <class TreeT>
static void BM_radius(benchmark::State &state) {
    using PointT = Point<float>;
    const auto bucket_size = state.range(0);

    auto points = loadMatterhorn();
    TreeT tree(bucket_size, 1.0f);
    tree.insert(points.begin(), points.end());

    const PointT query{0.0f, 0.0f, 0.0f};
    const float radius = 100.0f;

    for (auto _ : state) {
        auto results = tree.radiusSearch(query, radius);
        benchmark::DoNotOptimize(results);
    }
}

} // namespace

// Insert benchmarks for bucket_size 32 and 64
BENCHMARK_TEMPLATE(BM_insert, alloc_helpers::StandardTree<Point<float>>)->Arg(32)->Arg(64);
BENCHMARK_TEMPLATE(BM_insert, alloc_helpers::FastPoolTree<Point<float>>)->Arg(32)->Arg(64);
BENCHMARK_TEMPLATE(BM_insert, alloc_helpers::PoolTree<Point<float>>)->Arg(32)->Arg(64);
BENCHMARK_TEMPLATE(BM_insert, alloc_helpers::PreallocatedPoolTree<Point<float>>)->Arg(32)->Arg(64);

// KNN search benchmarks for bucket_size 32 and 64
BENCHMARK_TEMPLATE(BM_knn, alloc_helpers::StandardTree<Point<float>>)->Arg(32)->Arg(64);
BENCHMARK_TEMPLATE(BM_knn, alloc_helpers::FastPoolTree<Point<float>>)->Arg(32)->Arg(64);
BENCHMARK_TEMPLATE(BM_knn, alloc_helpers::PoolTree<Point<float>>)->Arg(32)->Arg(64);
BENCHMARK_TEMPLATE(BM_knn, alloc_helpers::PreallocatedPoolTree<Point<float>>)->Arg(32)->Arg(64);

// Radius search benchmarks for bucket_size 32 and 64
BENCHMARK_TEMPLATE(BM_radius, alloc_helpers::StandardTree<Point<float>>)->Arg(32)->Arg(64);
BENCHMARK_TEMPLATE(BM_radius, alloc_helpers::FastPoolTree<Point<float>>)->Arg(32)->Arg(64);
BENCHMARK_TEMPLATE(BM_radius, alloc_helpers::PoolTree<Point<float>>)->Arg(32)->Arg(64);
BENCHMARK_TEMPLATE(BM_radius, alloc_helpers::PreallocatedPoolTree<Point<float>>)->Arg(32)->Arg(64);
