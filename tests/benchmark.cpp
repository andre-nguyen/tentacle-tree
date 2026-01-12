#include "point.h"
#include "tt/impl/asc_io.hpp"
#include "tt/tentacle_tree.hpp"
#include <benchmark/benchmark.h>

static void BM_load(benchmark::State &state) {
    auto cloud = tt::loadASC<float>("matterhorn.asc");
    std::vector<Point<float>> points;
    points.reserve(cloud.coords.size());
    std::ranges::transform(cloud.coords, std::back_inserter(points),
                           [](const std::array<float, 3> &point) {
                               return Point<float>{point[0], point[1], point[2]};
                           });
    for (auto _ : state) {
        tt::TentacleTree<Point<float>> tree(128, 1);
        tree.insert(points.begin(), points.end());
    }
}
// Register the function as a benchmark
BENCHMARK(BM_load);