#include "point.h"
#include "rerun.h"
#include "tt/impl/asc_io.hpp"
#include "tt/tentacle_tree.hpp"
#include <benchmark/benchmark.h>


std::vector<Point<float>> loadCloud(const std::filesystem::path &path) {
    auto cloud = tt::loadASC<float>("matterhorn.asc");
    std::vector<Point<float>> points;
    points.reserve(cloud.coords.size());
    std::ranges::transform(cloud.coords, std::back_inserter(points),
                           [](const std::array<float, 3> &point) {
                               return Point<float>{point[0], point[1], point[2]};
                           });
    return points;
}

template <class... Args>
static void BM_insert(benchmark::State &state, Args &&...args) {
    auto args_tuple = std::make_tuple(std::move(args)...);
    auto points = loadCloud("matterhorn.asc");
    for (auto _ : state) {
        const auto bucket_size = std::get<0>(args_tuple);
        const auto minimum_extent = std::get<1>(args_tuple);
        tt::TentacleTree<Point<float>> tree(bucket_size, minimum_extent);
        tree.insert(points.begin(), points.end());
    }
}

BENCHMARK_CAPTURE(BM_insert, "insert", 128, 1.0f)->Name("TentacleTree::insert/128,1.0f");
BENCHMARK_CAPTURE(BM_insert, "insert", 64, 2.0f)->Name("TentacleTree::insert/ 64,2.0f");
BENCHMARK_CAPTURE(BM_insert, "insert", 32, 0.5f)->Name("TentacleTree::insert/ 32,0.5f");

template <class... Args>
static void BM_delete(benchmark::State &state, Args &&...args) {
    auto points = loadCloud("matterhorn.asc");

    // const auto rec = rerun::RecordingStream("load asc");
    // rec.spawn().exit_on_failure();
    // rec.set_time_sequence("time", 0);
    constexpr std::size_t kBucketSize = 128;
    constexpr std::size_t kMinimumExtent = 1.0f;

    auto args_tuple = std::make_tuple(std::move(args)...);
    for (auto _ : state) {
        state.PauseTiming();
        tt::TentacleTree<Point<float>> tree(kBucketSize, kMinimumExtent);
        tree.insert(points.begin(), points.end());
        const auto bounding_box = std::get<0>(args_tuple);
        state.ResumeTiming();
        tree.boxDelete(bounding_box);
    }

    // auto rerun_points = rerun::Points3D().with_positions(toRerunPositions<float>(points));
    // rec.log("original_points", rerun_points);
    // rec.set_time_sequence("time", 1);

    tt::TentacleTree<Point<float>> tree(kBucketSize, kMinimumExtent);
    tree.insert(points.begin(), points.end());
    const auto bounding_box = std::get<0>(args_tuple);
    tree.boxDelete(bounding_box);

    // rerun_points = rerun::Points3D().with_positions(toRerunPositions<float>(tree));
    // rec.log("tree", rerun_points);
}

BENCHMARK_CAPTURE(BM_delete, "delete", tt::BoundingBox<float>{{-200, -200, -200}, {200, 200, 200}})
    ->Name("TentacleTree::boxDelete/200x200");
BENCHMARK_CAPTURE(BM_delete, "delete", tt::BoundingBox<float>{
    {394'081, 5'091'000, -200}, {397'000, 5'093'000, 4000}})
    ->Name("TentacleTree::boxDelete/mountain");
