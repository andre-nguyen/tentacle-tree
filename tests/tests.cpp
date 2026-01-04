#include <tt/impl/bounded_priority_queue.hpp>
#include <tt/tentacle_tree.hpp>

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <functional>
#include <optional>
#include <random>
#include <rerun.hpp>

template <typename FloatT>
struct Point {
    using Float = FloatT;
    using value_type = FloatT; // For compatibility with STL
    [[nodiscard]] FloatT x() const { return coords[0]; }
    [[nodiscard]] FloatT y() const { return coords[1]; }
    [[nodiscard]] FloatT z() const { return coords[2]; }
    [[nodiscard]] FloatT operator[](std::size_t idx) const { return coords[idx]; }
    [[nodiscard]] FloatT &operator[](std::size_t idx) { return coords[idx]; }
    FloatT coords[3];

    Point<FloatT> &operator+=(const Point<FloatT> &other) {
        coords[0] += other.coords[0];
        coords[1] += other.coords[1];
        coords[2] += other.coords[2];
        return *this;
    }
};

TEST_CASE("computeBoundingBox") {
    {
        std::vector<Point<float>> points = {
            {{1.0f, 2.0f, 3.0f}},
            {{-1.0f, 0.0f, 4.0f}},
            {{0.5f, -2.0f, 1.0f}},
        };

        auto [min_coords, max_coords] = tt::impl::computeBoundingBox(points.begin(), points.end());

        CHECK(min_coords[0] == doctest::Approx(-1.0f));
        CHECK(min_coords[1] == doctest::Approx(-2.0f));
        CHECK(min_coords[2] == doctest::Approx(1.0f));

        CHECK(max_coords[0] == doctest::Approx(1.0f));
        CHECK(max_coords[1] == doctest::Approx(2.0f));
        CHECK(max_coords[2] == doctest::Approx(4.0f));
    }
    {
        std::vector<Point<double>> points = {
            {{10.0, 20.0, 30.0}},
            {{-10.0, 0.0, 40.0}},
            {{5.0, -20.0, 10.0}},
        };

        auto [min_coords, max_coords] = tt::impl::computeBoundingBox(points.begin(), points.end());

        CHECK(min_coords[0] == doctest::Approx(-10.0));
        CHECK(min_coords[1] == doctest::Approx(-20.0));
        CHECK(min_coords[2] == doctest::Approx(10.0));

        CHECK(max_coords[0] == doctest::Approx(10.0));
        CHECK(max_coords[1] == doctest::Approx(20.0));
        CHECK(max_coords[2] == doctest::Approx(40.0));
    }
    {
        std::vector<Point<int>> points = {
            {{100, 200, 300}},
            {{-100, 0, 400}},
            {{50, -200, 100}},
        };

        auto [min_coords, max_coords] = tt::impl::computeBoundingBox(points.begin(), points.end());

        CHECK(min_coords[0] == -100);
        CHECK(min_coords[1] == -200);
        CHECK(min_coords[2] == 100);

        CHECK(max_coords[0] == 100);
        CHECK(max_coords[1] == 200);
        CHECK(max_coords[2] == 400);
    }
}

// Original implementation from the Octree paper
// https://github.com/jbehley/octree/blob/8e3927d48d5ce61f94aab6090d77b4434f87dc89/Octree.hpp#L479-L488
auto originalComputeCenterAndMaxExtent(const std::array<float, 3> &min_coords,
                                       const std::array<float, 3> &max_coords) {
    std::array<float, 3> center = min_coords;
    float max_extent = 0.5f * (max_coords[0] - min_coords[0]);
    center[0] += max_extent;
    for (std::size_t i = 1; i < 3; ++i) {
        float extent = 0.5f * (max_coords[i] - min_coords[i]);
        center[i] += extent;
        if (extent > max_extent) {
            max_extent = extent;
        }
    }
    return std::make_pair(center, max_extent);
}

TEST_CASE("computeCenter") {
    {
        // Seems like the implementation is derived from the original Octree paper. Thus we check
        // that we get the same result as the original.
        const std::array<float, 3> min_coords = {-1.0f, -2.0f, 1.0f};
        const std::array<float, 3> max_coords = {1.0f, 2.0f, 4.0f};

        tt::BoundingBox<float> bbox_f{min_coords, max_coords};
        auto [center, max_extent] = tt::impl::computeCenterAndMaxExtent(bbox_f);
        auto [orig_center, orig_max_extent] =
            originalComputeCenterAndMaxExtent(min_coords, max_coords);

        CHECK(center[0] == doctest::Approx(orig_center[0]));
        CHECK(center[1] == doctest::Approx(orig_center[1]));
        CHECK(center[2] == doctest::Approx(orig_center[2]));
        CHECK(max_extent == doctest::Approx(orig_max_extent));
    }
    {
        std::array<double, 3> min_coords = {-10.0, -20.0, 10.0};
        std::array<double, 3> max_coords = {10.0, 20.0, 40.0};
        tt::BoundingBox<double> bbox_d{min_coords, max_coords};
        auto [center, max_extent] = tt::impl::computeCenterAndMaxExtent(bbox_d);

        CHECK(center[0] == doctest::Approx(0.0));
        CHECK(center[1] == doctest::Approx(0.0));
        CHECK(center[2] == doctest::Approx(25.0));
        CHECK(max_extent == doctest::Approx(20.0));
    }
}

TEST_CASE("computeMortonCode") {
    std::array<float, 3> center = {0.0f, 0.0f, 0.0f};
    Point<float> p0 = {{1, 1, 1}};
    Point<float> p1 = {{-1, 1, 1}};
    Point<float> p2 = {{-1, -1, 1}};
    Point<float> p3 = {{1, -1, 1}};
    Point<float> p4 = {{1, 1, -1}};
    Point<float> p5 = {{-1, 1, -1}};
    Point<float> p6 = {{-1, -1, -1}};
    Point<float> p7 = {{1, -1, -1}};
    {
        auto code = tt::impl::computeMortonCode<Point<float>>(p0, center);
        CHECK(code == 0b0111);
        code = tt::impl::computeMortonCode<Point<float>>(p1, center);
        CHECK(code == 0b0110);
        code = tt::impl::computeMortonCode<Point<float>>(p2, center);
        CHECK(code == 0b0100);
        code = tt::impl::computeMortonCode<Point<float>>(p3, center);
        CHECK(code == 0b0101);
        code = tt::impl::computeMortonCode<Point<float>>(p4, center);
        CHECK(code == 0b0011);
        code = tt::impl::computeMortonCode<Point<float>>(p5, center);
        CHECK(code == 0b0010);
        code = tt::impl::computeMortonCode<Point<float>>(p6, center);
        CHECK(code == 0b0000);
        code = tt::impl::computeMortonCode<Point<float>>(p7, center);
        CHECK(code == 0b0001);
    }
    Point<float> offset = {{100.0f, -2.0f, 50.0f}};
    center[0] += offset[0];
    center[1] += offset[1];
    center[2] += offset[2];
    p0 += offset;
    p1 += offset;
    p2 += offset;
    p3 += offset;
    p4 += offset;
    p5 += offset;
    p6 += offset;
    p7 += offset;
    {
        auto code = tt::impl::computeMortonCode<Point<float>>(p0, center);
        CHECK(code == 0b0111);
        code = tt::impl::computeMortonCode<Point<float>>(p1, center);
        CHECK(code == 0b0110);
        code = tt::impl::computeMortonCode<Point<float>>(p2, center);
        CHECK(code == 0b0100);
        code = tt::impl::computeMortonCode<Point<float>>(p3, center);
        CHECK(code == 0b0101);
        code = tt::impl::computeMortonCode<Point<float>>(p4, center);
        CHECK(code == 0b0011);
        code = tt::impl::computeMortonCode<Point<float>>(p5, center);
        CHECK(code == 0b0010);
        code = tt::impl::computeMortonCode<Point<float>>(p6, center);
        CHECK(code == 0b0000);
        code = tt::impl::computeMortonCode<Point<float>>(p7, center);
        CHECK(code == 0b0001);
    }
}

TEST_CASE("distributePointsToOctants") {
    // Test with center at origin
    {
        std::array<float, 3> center = {0.0f, 0.0f, 0.0f};

        // Create 8 points, one in each octant
        std::vector<Point<float>> points = {
            {{1, 1, 1}},    // octant 0b0111 = 7
            {{-1, 1, 1}},   // octant 0b0110 = 6
            {{-1, -1, 1}},  // octant 0b0100 = 4
            {{1, -1, 1}},   // octant 0b0101 = 5
            {{1, 1, -1}},   // octant 0b0011 = 3
            {{-1, 1, -1}},  // octant 0b0010 = 2
            {{-1, -1, -1}}, // octant 0b0000 = 0
            {{1, -1, -1}},  // octant 0b0001 = 1
        };

        auto octant_points =
            tt::impl::distributePointsToOctants<Point<float>>(points.begin(), points.end(), center);

        // Check that each octant has exactly one point
        CHECK(octant_points[0].size() == 1);
        CHECK(octant_points[1].size() == 1);
        CHECK(octant_points[2].size() == 1);
        CHECK(octant_points[3].size() == 1);
        CHECK(octant_points[4].size() == 1);
        CHECK(octant_points[5].size() == 1);
        CHECK(octant_points[6].size() == 1);
        CHECK(octant_points[7].size() == 1);

        // Check that the points are in the correct octants
        CHECK(octant_points[7][0].x() == doctest::Approx(1.0f));
        CHECK(octant_points[7][0].y() == doctest::Approx(1.0f));
        CHECK(octant_points[7][0].z() == doctest::Approx(1.0f));

        CHECK(octant_points[6][0].x() == doctest::Approx(-1.0f));
        CHECK(octant_points[6][0].y() == doctest::Approx(1.0f));
        CHECK(octant_points[6][0].z() == doctest::Approx(1.0f));

        CHECK(octant_points[4][0].x() == doctest::Approx(-1.0f));
        CHECK(octant_points[4][0].y() == doctest::Approx(-1.0f));
        CHECK(octant_points[4][0].z() == doctest::Approx(1.0f));

        CHECK(octant_points[5][0].x() == doctest::Approx(1.0f));
        CHECK(octant_points[5][0].y() == doctest::Approx(-1.0f));
        CHECK(octant_points[5][0].z() == doctest::Approx(1.0f));

        CHECK(octant_points[3][0].x() == doctest::Approx(1.0f));
        CHECK(octant_points[3][0].y() == doctest::Approx(1.0f));
        CHECK(octant_points[3][0].z() == doctest::Approx(-1.0f));

        CHECK(octant_points[2][0].x() == doctest::Approx(-1.0f));
        CHECK(octant_points[2][0].y() == doctest::Approx(1.0f));
        CHECK(octant_points[2][0].z() == doctest::Approx(-1.0f));

        CHECK(octant_points[0][0].x() == doctest::Approx(-1.0f));
        CHECK(octant_points[0][0].y() == doctest::Approx(-1.0f));
        CHECK(octant_points[0][0].z() == doctest::Approx(-1.0f));

        CHECK(octant_points[1][0].x() == doctest::Approx(1.0f));
        CHECK(octant_points[1][0].y() == doctest::Approx(-1.0f));
        CHECK(octant_points[1][0].z() == doctest::Approx(-1.0f));
    }

    // Test with offset center
    {
        std::array<float, 3> center = {10.0f, 20.0f, 30.0f};

        std::vector<Point<float>> points = {
            {{11, 21, 31}}, // octant 0b0111 = 7 (all positive relative to center)
            {{9, 21, 31}},  // octant 0b0110 = 6 (x negative, y,z positive)
            {{9, 19, 31}},  // octant 0b0100 = 4 (x,y negative, z positive)
            {{11, 19, 31}}, // octant 0b0101 = 5 (x,z positive, y negative)
            {{11, 21, 29}}, // octant 0b0011 = 3 (x,y positive, z negative)
            {{9, 21, 29}},  // octant 0b0010 = 2 (y positive, x,z negative)
            {{9, 19, 29}},  // octant 0b0000 = 0 (all negative relative to center)
            {{11, 19, 29}}, // octant 0b0001 = 1 (x positive, y,z negative)
        };

        auto octant_points =
            tt::impl::distributePointsToOctants<Point<float>>(points.begin(), points.end(), center);

        // Check that each octant has exactly one point
        for (std::size_t i = 0; i < 8; ++i) {
            CHECK(octant_points[i].size() == 1);
        }

        // Verify correct distribution (checking a few key octants)
        CHECK(octant_points[7][0].x() > center[0]);
        CHECK(octant_points[7][0].y() > center[1]);
        CHECK(octant_points[7][0].z() > center[2]);

        CHECK(octant_points[0][0].x() < center[0]);
        CHECK(octant_points[0][0].y() < center[1]);
        CHECK(octant_points[0][0].z() < center[2]);
    }

    // Test with multiple points in same octant
    {
        std::array<float, 3> center = {0.0f, 0.0f, 0.0f};

        std::vector<Point<float>> points = {
            {{1, 1, 1}},       // octant 7
            {{2, 2, 2}},       // octant 7
            {{0.5, 0.5, 0.5}}, // octant 7
            {{-1, -1, -1}},    // octant 0
            {{-2, -2, -2}},    // octant 0
        };

        auto octant_points =
            tt::impl::distributePointsToOctants<Point<float>>(points.begin(), points.end(), center);

        // Check that octant 7 has 3 points
        CHECK(octant_points[7].size() == 3);

        // Check that octant 0 has 2 points
        CHECK(octant_points[0].size() == 2);

        // Check that other octants are empty
        for (std::size_t i = 1; i < 7; ++i) {
            CHECK(octant_points[i].size() == 0);
        }
    }

    // Test with empty input
    {
        std::array<float, 3> center = {0.0f, 0.0f, 0.0f};
        std::vector<Point<float>> points;

        auto octant_points =
            tt::impl::distributePointsToOctants<Point<float>>(points.begin(), points.end(), center);

        // All octants should be empty
        for (std::size_t i = 0; i < 8; ++i) {
            CHECK(octant_points[i].size() == 0);
        }
    }

    // Test with points on boundaries (should go to positive side)
    {
        std::array<float, 3> center = {0.0f, 0.0f, 0.0f};

        std::vector<Point<float>> points = {
            {{0.0f, 0.0f,
              0.0f}}, // exactly on center - should go to octant 0 (all comparisons fail)
            {{1.0f, 0.0f, 0.0f}}, // on x boundary - should go to octant with x positive
        };

        auto octant_points =
            tt::impl::distributePointsToOctants<Point<float>>(points.begin(), points.end(), center);

        // Point at center should be in octant 0
        CHECK(octant_points[0].size() == 1);

        // Point with x=1, y=0, z=0 should be in octant 0b0001 = 1
        CHECK(octant_points[1].size() == 1);
    }
}

TEST_CASE("computeOctantCenter") {
    // Test that child centers are parent_center +/- child_half_extent on each axis
    {
        std::array<double, 3> parent = {10.0, 20.0, 30.0};
        double parent_half = 8.0; // child half should be 4.0
        double expected_child_half = parent_half * 0.5;

        for (std::size_t idx = 0; idx < 8; ++idx) {
            auto child = tt::impl::computeOctantCenter(parent, parent_half, idx);

            // X-axis (bit 0)
            if (idx & 0b0001) {
                CHECK(child[0] == doctest::Approx(parent[0] + expected_child_half));
            } else {
                CHECK(child[0] == doctest::Approx(parent[0] - expected_child_half));
            }

            // Y-axis (bit 1)
            if (idx & 0b0010) {
                CHECK(child[1] == doctest::Approx(parent[1] + expected_child_half));
            } else {
                CHECK(child[1] == doctest::Approx(parent[1] - expected_child_half));
            }

            // Z-axis (bit 2)
            if (idx & 0b0100) {
                CHECK(child[2] == doctest::Approx(parent[2] + expected_child_half));
            } else {
                CHECK(child[2] == doctest::Approx(parent[2] - expected_child_half));
            }
        }
    }

    // Also check with float coordinates and non-integer half extent
    {
        std::array<float, 3> parent = {0.5f, -1.25f, 2.0f};
        float parent_half = 3.5f; // child half should be 1.75
        float expected_child_half = parent_half * 0.5f;

        // Check a couple of octants explicitly
        auto c0 = tt::impl::computeOctantCenter(parent, parent_half, 0); // all negative
        CHECK(c0[0] == doctest::Approx(parent[0] - expected_child_half));
        CHECK(c0[1] == doctest::Approx(parent[1] - expected_child_half));
        CHECK(c0[2] == doctest::Approx(parent[2] - expected_child_half));

        auto c7 = tt::impl::computeOctantCenter(parent, parent_half, 7); // all positive
        CHECK(c7[0] == doctest::Approx(parent[0] + expected_child_half));
        CHECK(c7[1] == doctest::Approx(parent[1] + expected_child_half));
        CHECK(c7[2] == doctest::Approx(parent[2] + expected_child_half));
    }
}

template <typename FloatT>
std::vector<rerun::Position3D> toRerunPositions(const std::vector<Point<FloatT>> &points) {
    std::vector<rerun::Position3D> rr_positions;
    rr_positions.reserve(points.size());
    for (const auto &p : points) {
        rr_positions.emplace_back(static_cast<double>(p.x()), static_cast<double>(p.y()),
                                  static_cast<double>(p.z()));
    }
    return rr_positions;
}

template <typename FloatT>
std::vector<rerun::Position3D> toRerunPositions(const tt::TentacleTree<Point<FloatT>> &tree) {
    std::vector<rerun::Position3D> rr_positions;
    const auto root = tree.root();
    const auto leaves = tt::impl::collectLeafNodes(*root);
    auto total = std::accumulate(leaves.cbegin(), leaves.cend(), std::size_t(0),
                                 [](std::size_t acc, const tt::Node<Point<FloatT>> *node) {
                                     return acc + node->points.size();
                                 });
    rr_positions.reserve(total);
    for (const auto *leaf : leaves) {
        for (const auto &p : leaf->points) {
            rr_positions.emplace_back(static_cast<double>(p.x()), static_cast<double>(p.y()),
                                      static_cast<double>(p.z()));
        }
    }
    return rr_positions;
}

template <tt::Point3d PointT>
void toRerunBoxes(const tt::Node<PointT> &node,
                  std::vector<rerun::components::PoseTranslation3D> &centers,
                  std::vector<rerun::components::HalfSize3D> &half_sizes,
                  const std::optional<rerun::Color> &color = std::nullopt) {
    centers.emplace_back(static_cast<float>(node.center[0]), static_cast<float>(node.center[1]),
                         static_cast<float>(node.center[2]));
    half_sizes.emplace_back(static_cast<float>(node.half_extent),
                            static_cast<float>(node.half_extent),
                            static_cast<float>(node.half_extent));
    for (const auto &child : node.children) {
        if (!child) {
            continue;
        }
        toRerunBoxes(*child, centers, half_sizes, color);
    }
}

template <tt::Point3d PointT>
rerun::Boxes3D toRerunBoxes(const tt::Node<PointT> &node,
                            const std::optional<rerun::Color> &color = std::nullopt) {
    std::vector<rerun::components::PoseTranslation3D> centers;
    std::vector<rerun::components::HalfSize3D> half_sizes;
    toRerunBoxes(node, centers, half_sizes, color);
    if (color.has_value()) {
        return rerun::Boxes3D::from_centers_and_half_sizes(centers, half_sizes).with_colors(*color);
    }
    return rerun::Boxes3D::from_centers_and_half_sizes(centers, half_sizes);
}

template <tt::Point3d PointT>
rerun::Boxes3D toRerunBoxes(const tt::TentacleTree<PointT> &tree,
                            const std::optional<rerun::Color> &color = std::nullopt) {
    const auto root = tree.root();
    return toRerunBoxes(*root, color);
}

template <typename FloatT>
rerun::Boxes3D toRerunBoxes(const tt::BoundingBox<FloatT> &bbox,
                            const std::optional<rerun::Color> &color = std::nullopt) {
    if (color.has_value()) {
        return rerun::Boxes3D::from_mins_and_sizes(
                   {{bbox.min_coords[0], bbox.min_coords[1], bbox.min_coords[2]}},
                   {{bbox.max_coords[0] - bbox.min_coords[0],
                     bbox.max_coords[1] - bbox.min_coords[1],
                     bbox.max_coords[2] - bbox.min_coords[2]}})
            .with_colors(*color);
    }
    return rerun::Boxes3D::from_mins_and_sizes(
        {{bbox.min_coords[0], bbox.min_coords[1], bbox.min_coords[2]}},
        {{bbox.max_coords[0] - bbox.min_coords[0], bbox.max_coords[1] - bbox.min_coords[1],
          bbox.max_coords[2] - bbox.min_coords[2]}});
}

TEST_CASE("initialize") {
    // clang-format off
    std::vector<Point<float>> points = {
        {1, 1, 1},
        {12, 12, 12},
        {1, 1, 1},
        {4, 1, 1},
        {5, 1, 1},
        {6, 1, 1},
        {7, 1, -1},
        {8, 13, 1},
        {9, 1, 1},
        {9, 1, 7},
        {1, 15, 1}
    };
    // clang-format on
    tt::TentacleTree<Point<float>> tree(1, 0.1f);
    tree.insert(points.begin(), points.end());

    // Verify the tree was constructed and points are present in leaf nodes
    const auto root = tree.root();
    REQUIRE(root != nullptr);

    // Collect leaf nodes
    std::vector<const tt::Node<Point<float>> *> leaves;
    std::function<void(const tt::Node<Point<float>> *)> collect_leaves;
    collect_leaves = [&](const tt::Node<Point<float>> *node) {
        bool is_leaf = true;
        for (const auto &child : node->children) {
            if (child) {
                is_leaf = false;
                collect_leaves(child.get());
            }
        }
        if (is_leaf)
            leaves.push_back(node);
    };
    collect_leaves(root);

    // Ensure we have at least one leaf and total point count matches input
    REQUIRE(!leaves.empty());
    std::size_t total_points_in_leaves = 0;
    for (const auto *leaf : leaves)
        total_points_in_leaves += leaf->points.size();
    CHECK(total_points_in_leaves == points.size());

    // For each leaf, compute its top-level octant relative to the root center
    const auto root_center = root->center;
    for (const auto *leaf : leaves) {
        std::size_t top_code = 0;
        if (leaf->center[0] > root_center[0])
            top_code |= 0b0001;
        if (leaf->center[1] > root_center[1])
            top_code |= 0b0010;
        if (leaf->center[2] > root_center[2])
            top_code |= 0b0100;

        // Each point stored in this leaf must belong to the same top-level octant
        for (const auto &p : leaf->points) {
            auto code = tt::impl::computeMortonCode<Point<float>>(p, root_center);
            CHECK(code == top_code);
        }
    }

    // Keep the visualization logging (optional)
    // const auto rec = rerun::RecordingStream("rerun_example_box3d_batch");
    // rec.spawn().exit_on_failure();
    //
    // rec.log("points", rerun::Points3D(toRerunPositions(points)));
    // rec.log("boxes", toRerunBoxes(tree));
}

// I'm not sure if this is expected behavior according to the algorithm but if we have a tree of
// bucket size 2 and insert 2 points in the same location, the tree does not split down to min
// extent size. But if you add a 3rd point in the same location, it does.
// This makes for the odd case where a leaf isn't necessarily a node of size min_extent.
// TEST_CASE("init two points") {
//     std::vector<Point<double>> points = {{{-10.1, -10.0, -10.0}},
//                                          {{-10.1, -10.0, -10.0}},
//                                          {{-10.1, -10.0, -10.0}},
//                                          {{10.5, 10.0, 10.0}},
//                                          {{10.5, 10.0, 10.0}}};
//
//     tt::TentacleTree<Point<double>> tree(2, 0.01);
//     tree.insert(points.begin(), points.end());
//
//     const auto root = tree.root();
//     REQUIRE(root != nullptr);
//     // CHECK(root->points.size() == 0); // should have split
//     //
//     // // Check that each child has one point
//     // std::size_t total_points = 0;
//     // for (const auto &child : root->children) {
//     //     if (child) {
//     //         total_points += child->points.size();
//     //         CHECK(child->points.size() == 1);
//     //     }
//     // }
//     // CHECK(total_points == points.size());
//
//     const auto rec = rerun::RecordingStream("tentacle_tree_two_points");
//     rec.spawn().exit_on_failure();
//     rec.log("points", rerun::Points3D(toRerunPositions(points)));
//     rec.log("tree", toRerunBoxes(tree, rerun::Color(0, 255, 0)));
// }

template <typename FloatT>
class PointCloudTestFixture {
  public:
    PointCloudTestFixture() : points_(generateCube()) {}

    static std::vector<Point<FloatT>> generateCube(const std::array<FloatT, 3> &center = {
                                                       FloatT(0), FloatT(0), FloatT(0)}) {
        // Generate equally distributed points in a cube of side length 2 centered at 'center'
        std::vector<Point<FloatT>> points;
        points.reserve(64);
        const auto spacing = FloatT(2.0 / 3.0);
        const FloatT start_x = center[0] - FloatT(1.0);
        const FloatT start_y = center[1] - FloatT(1.0);
        const FloatT start_z = center[2] - FloatT(1.0);

        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                for (int k = 0; k < 4; ++k) {
                    points.push_back(Point<FloatT>{{start_x + static_cast<FloatT>(i) * spacing,
                                                    start_y + static_cast<FloatT>(j) * spacing,
                                                    start_z + static_cast<FloatT>(k) * spacing}});
                }
            }
        }
        return points;
    }

  protected:
    std::vector<Point<FloatT>> points_;
};

template <typename FloatT>
bool isClose(FloatT a, FloatT b, const FloatT epsilon = static_cast<FloatT>(1e-12)) {
    return std::abs(a - b) < epsilon;
}

TEST_CASE_FIXTURE(PointCloudTestFixture<double>, "addPointsOutside") {
    tt::TentacleTree<Point<double>> tree(1, 0.01);
    tree.insert(points_.begin(), points_.end());

    {
        auto leaves = tt::impl::collectLeafNodes<Point<double>>(*tree.root());
        REQUIRE(leaves.size() == 4 * 4 * 4);
        std::vector<bool> point_is_found(points_.size(), false);
        for (const auto &leaf : leaves) {
            CHECK(leaf->points.size() == 1);
            auto it = std::find_if(points_.begin(), points_.end(),
                                   [q = &leaf->points[0]](const Point<double> &p) {
                                       return isClose(p.x(), q->x()) && isClose(p.y(), q->y()) &&
                                              isClose(p.z(), q->z());
                                   });
            REQUIRE(it != points_.end());
            auto index = static_cast<std::size_t>(std::distance(points_.begin(), it));
            CHECK(!point_is_found[index]);
            point_is_found[index] = true;
        }
    }

    // Now add points outside the initial bounding box
    std::array<double, 3> offset = {10.0, -15.0, 20.0};
    auto outside_points = generateCube(offset);
    tree.insert(outside_points.begin(), outside_points.end());
    {
        // Check that the original points are there AND the new ones
        auto leaves = tt::impl::collectLeafNodes<Point<double>>(*tree.root());
        REQUIRE(leaves.size() == 2 * 4 * 4 * 4); // original + new points
        std::vector<bool> point_is_found(points_.size() + outside_points.size(), false);
        for (const auto &leaf : leaves) {
            CHECK(leaf->points.size() == 1);
            auto &p = leaf->points[0];
            auto it =
                std::find_if(points_.begin(), points_.end(), [q = &p](const Point<double> &pt) {
                    return isClose(pt.x(), q->x()) && isClose(pt.y(), q->y()) &&
                           isClose(pt.z(), q->z());
                });
            if (it != points_.end()) {
                auto index = static_cast<std::size_t>(std::distance(points_.begin(), it));
                CHECK(!point_is_found[index]);
                point_is_found[index] = true;
                continue;
            }
            it = std::find_if(outside_points.begin(), outside_points.end(),
                              [q = &p](const Point<double> &pt) {
                                  return isClose(pt.x(), q->x()) && isClose(pt.y(), q->y()) &&
                                         isClose(pt.z(), q->z());
                              });
            REQUIRE(it != outside_points.end());
            auto index = static_cast<std::size_t>(std::distance(outside_points.begin(), it)) +
                         points_.size();
            CHECK(!point_is_found[index]);
            point_is_found[index] = true;
        }
    }
}

TEST_CASE_FIXTURE(PointCloudTestFixture<double>, "overflowBuckets") {
    tt::TentacleTree<Point<double>> tree(3, 0.01);
    tree.insert(points_.begin(), points_.end());
    tree.insert(points_.begin(), points_.end());
    tree.insert(points_.begin(), points_.end());

    // const auto rec = rerun::RecordingStream("tentacle_tree_overflow_buckets");
    // rec.spawn().exit_on_failure();
    // rec.log("points", rerun::Points3D(toRerunPositions(points_)));
    // rec.log("tree", toRerunBoxes(tree, rerun::Color(0, 0, 255)));

    auto leaves = tt::impl::collectLeafNodes<Point<double>>(*tree.root());
    REQUIRE(leaves.size() == 4 * 4 * 4);
    for (const auto &leaf : leaves) {
        CHECK(leaf->points.size() == 3);
    }

    // Insert more than the bucket size and check the leaves haven't changed
    tree.insert(points_.begin(), points_.end());
    leaves = tt::impl::collectLeafNodes<Point<double>>(*tree.root());
    REQUIRE(leaves.size() == 4 * 4 * 4);
    for (const auto &leaf : leaves) {
        CHECK(leaf->points.size() == 3);
    }
}

TEST_CASE_FIXTURE(PointCloudTestFixture<double>, "delete") {
    SUBCASE("Delete everything") {
        tt::TentacleTree<Point<double>> tree(1, 0.01);
        tree.insert(points_.begin(), points_.end());

        auto leaves = tt::impl::collectLeafNodes<Point<double>>(*tree.root());
        REQUIRE(leaves.size() == 4 * 4 * 4);

        tree.boxDelete({{-100, -100, -100}, {100, 100, 100}});
        REQUIRE(tree.root() == nullptr);
    }

    SUBCASE("Delete nothing") {
        tt::TentacleTree<Point<double>> tree(1, 0.01);
        tree.insert(points_.begin(), points_.end());

        auto leaves = tt::impl::collectLeafNodes<Point<double>>(*tree.root());
        REQUIRE(leaves.size() == 4 * 4 * 4);

        tree.boxDelete({{-100, -100, -100}, {-50, -50, -50}});
        leaves = tt::impl::collectLeafNodes<Point<double>>(*tree.root());
        REQUIRE(leaves.size() == 4 * 4 * 4);
    }

    SUBCASE("Delete half X") {
        tt::TentacleTree<Point<double>> tree(1, 0.01);
        tree.insert(points_.begin(), points_.end());
        auto leaves = tt::impl::collectLeafNodes<Point<double>>(*tree.root());
        REQUIRE(leaves.size() == 4 * 4 * 4);

        // const auto rec = rerun::RecordingStream("tentacle_tree_box_delete");
        // rec.spawn().exit_on_failure();
        // rec.set_time_sequence("time", 0);
        // rec.log("points", rerun::Points3D(toRerunPositions(tree)));
        // rec.log("tree_before_delete", toRerunBoxes(tree, rerun::Color(255, 0, 0)));

        tree.boxDelete({{-100, -100, -100}, {0, 100, 100}});
        leaves = tt::impl::collectLeafNodes<Point<double>>(*tree.root());
        REQUIRE(leaves.size() == 2 * 4 * 4);

        for (const auto *leaf : leaves) {
            for (const auto &p : leaf->points) {
                CHECK(p.x() > 0.0);
            }
        }

        // rec.set_time_sequence("time", 1);
        // rec.log("points", rerun::Points3D(toRerunPositions(tree)));
        // rec.log("tree_before_delete", toRerunBoxes(tree, rerun::Color(255, 0, 0)));
    }

    SUBCASE("Delete half Y") {
        tt::TentacleTree<Point<double>> tree(1, 0.01);
        tree.insert(points_.begin(), points_.end());
        auto leaves = tt::impl::collectLeafNodes<Point<double>>(*tree.root());
        REQUIRE(leaves.size() == 4 * 4 * 4);

        tree.boxDelete({{-100, -100, -100}, {100, 0, 100}});
        leaves = tt::impl::collectLeafNodes<Point<double>>(*tree.root());
        REQUIRE(leaves.size() == 2 * 4 * 4);

        for (const auto *leaf : leaves) {
            for (const auto &p : leaf->points) {
                CHECK(p.y() > 0.0);
            }
        }
    }

    SUBCASE("Delete half Z") {
        tt::TentacleTree<Point<double>> tree(1, 0.01);
        tree.insert(points_.begin(), points_.end());
        auto leaves = tt::impl::collectLeafNodes<Point<double>>(*tree.root());
        REQUIRE(leaves.size() == 4 * 4 * 4);

        tree.boxDelete({{-100, -100, -100}, {100, 100, 0}});
        leaves = tt::impl::collectLeafNodes<Point<double>>(*tree.root());
        REQUIRE(leaves.size() == 2 * 4 * 4);

        for (const auto *leaf : leaves) {
            for (const auto &p : leaf->points) {
                CHECK(p.z() > 0.0);
            }
        }
    }

    SUBCASE("Delete inside points (keep only boundary)") {
        tt::TentacleTree<Point<double>> tree(1, 0.01);
        tree.insert(points_.begin(), points_.end());
        auto leaves = tt::impl::collectLeafNodes<Point<double>>(*tree.root());
        REQUIRE(leaves.size() == 4 * 4 * 4);

        const auto rec = rerun::RecordingStream("tentacle_tree_delete_inside");
        rec.spawn().exit_on_failure();
        rec.set_time_sequence("frame", 0);
        rec.log("points", rerun::Points3D(toRerunPositions(tree)));
        rec.log("tree", toRerunBoxes(tree, rerun::Color(255, 0, 0)));

        // The cube is from -1.0 to 1.0 in each axis, spacing is 2/3
        // So boundary points are those with at least one coordinate == -1.0 or 1.0
        double min = -0.5, max = 0.5;
        tree.boxDelete({{min, min, min}, {max, max, max}});
        leaves = tt::impl::collectLeafNodes<Point<double>>(*tree.root());

        rec.set_time_sequence("frame", 1);
        rec.log("points", rerun::Points3D(toRerunPositions(tree)));
        rec.log("tree", toRerunBoxes(tree, rerun::Color(0, 255, 0)));

        // After deletion, all remaining points must have at least one coordinate == min or max
        std::size_t boundary_count = 0;
        for (const auto *leaf : leaves) {
            for (const auto &p : leaf->points) {
                bool on_boundary = isClose(p.x(), min) || isClose(p.x(), max) ||
                                   isClose(p.y(), min) || isClose(p.y(), max) ||
                                   isClose(p.z(), min) || isClose(p.z(), max);
                boundary_count++;
            }
        }
        // There are 56 boundary points in a 4x4x4 cube (cube minus 2x2x2 inside = 64-8=56)
        CHECK(boundary_count == 56);
    }

    SUBCASE("Delete inner points in a column through the cube") {
        tt::TentacleTree<Point<double>> tree(1, 0.01);
        tree.insert(points_.begin(), points_.end());
        auto leaves = tt::impl::collectLeafNodes<Point<double>>(*tree.root());
        REQUIRE(leaves.size() == 4 * 4 * 4);

        // const auto rec = rerun::RecordingStream("tentacle_tree_delete_inside");
        // rec.spawn().exit_on_failure();
        // rec.set_time_sequence("frame", 0);
        // rec.log("points", rerun::Points3D(toRerunPositions(tree)));
        // rec.log("tree", toRerunBoxes(tree, rerun::Color(255, 0, 0)));

        tree.boxDelete({{-0.5, -0.5, -100}, {0.5, 0.5, 100}});
        leaves = tt::impl::collectLeafNodes<Point<double>>(*tree.root());
        REQUIRE(leaves.size() == (4 * 3) * 4);

        // rec.set_time_sequence("frame", 1);
        // rec.log("points", rerun::Points3D(toRerunPositions(tree)));
        // rec.log("tree", toRerunBoxes(tree, rerun::Color(0, 255, 0)));
    }
}

TEST_CASE("isBoxInNode") {
    tt::Node<Point<float>> node;
    node.center = {0.0f, 0.0f, 0.0f};
    node.half_extent = 1.0f;

    using BBoxF = tt::BoundingBox<float>;

    // Fully inside
    BBoxF inside;
    inside.min_coords = {-0.5f, -0.5f, -0.5f};
    inside.max_coords = {0.5f, 0.5f, 0.5f};
    CHECK(tt::impl::isBoxInNode<Point<float>>(inside, node));

    // Fully outside (disjoint)
    BBoxF outside;
    outside.min_coords = {2.0f, 2.0f, 2.0f};
    outside.max_coords = {3.0f, 3.0f, 3.0f};
    CHECK(!tt::impl::isBoxInNode<Point<float>>(outside, node));

    // Touching at face (min == node_max on x)
    BBoxF touching;
    touching.min_coords = {1.0f, -0.5f, -0.5f};
    touching.max_coords = {2.0f, 0.5f, 0.5f};
    CHECK(!tt::impl::isBoxInNode<Point<float>>(touching, node));

    // Barely outside (just beyond node_max)
    BBoxF barely_out;
    barely_out.min_coords = {1.0001f, -0.5f, -0.5f};
    barely_out.max_coords = {2.0f, 0.5f, 0.5f};
    CHECK(!tt::impl::isBoxInNode<Point<float>>(barely_out, node));

    // Overlapping partially
    BBoxF overlap;
    overlap.min_coords = {0.5f, 0.5f, 0.5f};
    overlap.max_coords = {1.5f, 1.5f, 1.5f};
    CHECK(!tt::impl::isBoxInNode<Point<float>>(overlap, node));

    // Overlaps in x/y but separated in z
    BBoxF sep_z;
    sep_z.min_coords = {0.5f, 0.5f, 2.0f};
    sep_z.max_coords = {1.5f, 1.5f, 3.0f};
    CHECK(!tt::impl::isBoxInNode<Point<float>>(sep_z, node));
}

TEST_CASE("pointInNode") {
    tt::Node<Point<float>> node;
    node.center = {10.0f, -1.0f, 1.0f};
    node.half_extent = 1.0f;

    CHECK(tt::impl::isPointInNode({10.1f, -1.1f, 1.1f}, node));
    CHECK(tt::impl::isPointInNode({9.9f, -0.9f, 0.9f}, node));
    CHECK(!tt::impl::isPointInNode({16.0f, -1.0f, 1.0f}, node));
    CHECK(!tt::impl::isPointInNode({10.0f, -4.0f, 1.0f}, node));
    CHECK(!tt::impl::isPointInNode({10.0f, -1.0f, 3.0f}, node));
    CHECK(!tt::impl::isPointInNode({123.0f, 123.0f, 123.0f}, node));
}

TEST_CASE("isSphereInNode") {
    // Node centered at origin with half-extent 1.0
    // This node spans from (-1, -1, -1) to (1, 1, 1)
    tt::Node<Point<float>> node;
    node.center = {0.0f, 0.0f, 0.0f};
    node.half_extent = 1.0f;

    SUBCASE("Sphere fully inside node - centered") {
        Point<float> sphere_center{{0.0f, 0.0f, 0.0f}};
        // Sphere with radius 0.5 centered at origin is fully inside the node
        float sphere_radius = 0.5f;
        CHECK(tt::impl::isSphereInNode(node, sphere_center, sphere_radius));
    }

    SUBCASE("Sphere fully inside node - small radius") {
        Point<float> sphere_center{{0.0f, 0.0f, 0.0f}};
        // Very small sphere is definitely inside
        float sphere_radius = 0.1f;
        CHECK(tt::impl::isSphereInNode(node, sphere_center, sphere_radius));
    }

    SUBCASE("Sphere too large - extends beyond node") {
        Point<float> sphere_center{{0.0f, 0.0f, 0.0f}};
        // Sphere with radius 1.5 extends beyond the node
        float sphere_radius = 1.5f;
        CHECK_FALSE(tt::impl::isSphereInNode(node, sphere_center, sphere_radius));
    }

    SUBCASE("Sphere exactly fits - touching all faces") {
        Point<float> sphere_center{{0.0f, 0.0f, 0.0f}};
        // Sphere with radius 1.0 exactly touches all faces
        float sphere_radius = 1.0f;
        CHECK(tt::impl::isSphereInNode(node, sphere_center, sphere_radius));
    }

    SUBCASE("Sphere center offset - still inside") {
        Point<float> sphere_center{{0.5f, 0.0f, 0.0f}};
        // Sphere centered at x=0.5 with radius 0.5 touches the max face but doesn't extend beyond
        float sphere_radius = 0.5f;
        CHECK(tt::impl::isSphereInNode(node, sphere_center, sphere_radius));
    }

    SUBCASE("Sphere center offset - extends beyond") {
        Point<float> sphere_center{{0.5f, 0.0f, 0.0f}};
        // Sphere centered at x=0.5 with radius 0.6 extends beyond the max face
        float sphere_radius = 0.6f;
        CHECK_FALSE(tt::impl::isSphereInNode(node, sphere_center, sphere_radius));
    }

    SUBCASE("Sphere center at node corner - extends beyond") {
        Point<float> sphere_center{{1.0f, 1.0f, 1.0f}};
        // Any sphere at a corner will extend beyond the node
        float sphere_radius = 0.1f;
        CHECK_FALSE(tt::impl::isSphereInNode(node, sphere_center, sphere_radius));
    }

    SUBCASE("Sphere center near boundary - barely fits") {
        Point<float> sphere_center{{0.8f, 0.0f, 0.0f}};
        // Sphere with radius 0.2 centered at x=0.8 just fits (0.8 + 0.2 = 1.0)
        float sphere_radius = 0.2f;
        CHECK(tt::impl::isSphereInNode(node, sphere_center, sphere_radius));
    }

    SUBCASE("Sphere center near boundary - slightly too large") {
        Point<float> sphere_center{{0.8f, 0.0f, 0.0f}};
        // Sphere with radius 0.21 extends beyond (0.8 + 0.21 = 1.01 > 1.0)
        float sphere_radius = 0.21f;
        CHECK_FALSE(tt::impl::isSphereInNode(node, sphere_center, sphere_radius));
    }

    SUBCASE("Sphere center outside node") {
        Point<float> sphere_center{{2.0f, 0.0f, 0.0f}};
        // Sphere center is outside the node
        float sphere_radius = 0.5f;
        CHECK_FALSE(tt::impl::isSphereInNode(node, sphere_center, sphere_radius));
    }

    SUBCASE("Zero radius sphere - always inside if center is inside") {
        Point<float> sphere_center{{0.0f, 0.0f, 0.0f}};
        float sphere_radius = 0.0f;
        CHECK(tt::impl::isSphereInNode(node, sphere_center, sphere_radius));
    }

    SUBCASE("Zero radius sphere - outside if center is outside") {
        Point<float> sphere_center{{1.5f, 0.0f, 0.0f}};
        float sphere_radius = 0.0f;
        CHECK_FALSE(tt::impl::isSphereInNode(node, sphere_center, sphere_radius));
    }

    SUBCASE("Different coordinate types - double") {
        tt::Node<Point<double>> node_d;
        node_d.center = {0.0, 0.0, 0.0};
        node_d.half_extent = 1.0;

        Point<double> sphere_center{{0.0, 0.0, 0.0}};
        double sphere_radius = 0.5;
        CHECK(tt::impl::isSphereInNode(node_d, sphere_center, sphere_radius));
    }

    SUBCASE("Asymmetric sphere position - all axes") {
        // Node: center=(0,0,0), half_extent=1.0, so bounds are [-1, 1] on all axes
        // Test 1: Sphere that clearly fits
        Point<float> sphere_center{{0.0f, -0.5f, 0.0f}};
        float sphere_radius = 0.4f;
        // y-axis check: center=-0.5, radius=0.4
        //   sphere_min = -0.5 - 0.4 = -0.9 (>= -1.0) ✓
        //   sphere_max = -0.5 + 0.4 = -0.1 (<= 1.0) ✓
        CHECK(tt::impl::isSphereInNode(node, sphere_center, sphere_radius));

        // Test 2: Sphere that exactly touches max boundary on y-axis
        sphere_center = {{0.0f, 0.3f, 0.0f}};
        sphere_radius = 0.7f;
        // y-axis check: center=0.3, radius=0.7
        //   sphere_min = 0.3 - 0.7 = -0.4 (>= -1.0) ✓
        //   sphere_max = 0.3 + 0.7 =  1.0 (<= 1.0) ✓ (exactly touches)
        CHECK(tt::impl::isSphereInNode(node, sphere_center, sphere_radius));

        // Test 3: Sphere that extends beyond max boundary on y-axis
        sphere_center = {{0.0f, 0.3f, 0.0f}};
        sphere_radius = 0.71f;
        // y-axis check: center=0.3, radius=0.71
        //   sphere_min = 0.3 - 0.71 = -0.41 (>= -1.0) ✓
        //   sphere_max = 0.3 + 0.71 =  1.01 (> 1.0) ✗ (extends beyond!)
        CHECK_FALSE(tt::impl::isSphereInNode(node, sphere_center, sphere_radius));

        // Test 4: Sphere that extends beyond max on x and z but fits on y
        sphere_center = {{0.9f, 0.0f, 0.9f}};
        sphere_radius = 0.2f;
        // x-axis check: center=0.9, radius=0.2
        //   sphere_min = 0.9 - 0.2 = 0.7 (>= -1.0) ✓
        //   sphere_max = 0.9 + 0.2 = 1.1 (> 1.0) ✗ (extends beyond!)
        CHECK_FALSE(tt::impl::isSphereInNode(node, sphere_center, sphere_radius));
    }

    SUBCASE("Large node - small sphere") {
        tt::Node<Point<float>> large_node;
        large_node.center = {0.0f, 0.0f, 0.0f};
        large_node.half_extent = 10.0f;

        Point<float> sphere_center{{0.0f, 0.0f, 0.0f}};
        float sphere_radius = 5.0f;
        CHECK(tt::impl::isSphereInNode(large_node, sphere_center, sphere_radius));
    }
}

TEST_CASE("isBoxOverlappingNode") {
    tt::Node<Point<float>> node;
    node.center = {0.0f, 0.0f, 0.0f};
    node.half_extent = 1.0f;

    using BBoxF = tt::BoundingBox<float>;

    // Fully inside
    BBoxF inside;
    inside.min_coords = {-0.5f, -0.5f, -0.5f};
    inside.max_coords = {0.5f, 0.5f, 0.5f};
    CHECK(tt::impl::isBoxOverlappingNode<Point<float>>(inside, node));

    // Fully outside (disjoint)
    BBoxF outside;
    outside.min_coords = {2.0f, 2.0f, 2.0f};
    outside.max_coords = {3.0f, 3.0f, 3.0f};
    CHECK(!tt::impl::isBoxOverlappingNode<Point<float>>(outside, node));

    // Touching at face (min == node_max on x)
    BBoxF touching;
    touching.min_coords = {1.0f, -0.5f, -0.5f};
    touching.max_coords = {2.0f, 0.5f, 0.5f};
    CHECK(!tt::impl::isBoxOverlappingNode<Point<float>>(touching, node));

    // Barely outside (just beyond node_max)
    BBoxF barely_out;
    barely_out.min_coords = {1.0001f, -0.5f, -0.5f};
    barely_out.max_coords = {2.0f, 0.5f, 0.5f};
    CHECK(!tt::impl::isBoxOverlappingNode<Point<float>>(barely_out, node));

    // Overlapping partially
    BBoxF overlap;
    overlap.min_coords = {0.5f, 0.5f, 0.5f};
    overlap.max_coords = {1.5f, 1.5f, 1.5f};
    CHECK(tt::impl::isBoxOverlappingNode<Point<float>>(overlap, node));

    // Overlaps in x/y but separated in z
    BBoxF sep_z;
    sep_z.min_coords = {0.5f, 0.5f, 2.0f};
    sep_z.max_coords = {1.5f, 1.5f, 3.0f};
    CHECK(!tt::impl::isBoxOverlappingNode<Point<float>>(sep_z, node));

    // Enclosing node
    BBoxF enclosing;
    enclosing.min_coords = {-2.0f, -2.0f, -2.0f};
    enclosing.max_coords = {2.0f, 2.0f, 2.0f};
    CHECK(tt::impl::isBoxOverlappingNode<Point<float>>(enclosing, node));
}

TEST_CASE("computeParentCenter") {
    tt::Node<Point<float>> node;
    node.center = {0.0f, 0.0f, 0.0f};
    node.half_extent = 1.0f;

    {
        const tt::Array3f target = {2.0f, 2.0f, 2.0f};
        const auto new_center = tt::impl::computeParentCenter(node, target);
        CHECK(new_center[0] == doctest::Approx(1.f));
        CHECK(new_center[1] == doctest::Approx(1.f));
        CHECK(new_center[2] == doctest::Approx(1.f));
    }
    {
        const tt::Array3f target = {100.0f, 300.0f, 200.0f};
        const auto new_center = tt::impl::computeParentCenter(node, target);
        CHECK(new_center[0] == doctest::Approx(1.f));
        CHECK(new_center[1] == doctest::Approx(1.f));
        CHECK(new_center[2] == doctest::Approx(1.f));
    }
    {
        const tt::Array3f target = {-100.0f, -300.0f, -200.0f};
        const auto new_center = tt::impl::computeParentCenter(node, target);
        CHECK(new_center[0] == doctest::Approx(-1.f));
        CHECK(new_center[1] == doctest::Approx(-1.f));
        CHECK(new_center[2] == doctest::Approx(-1.f));
    }
    {
        const tt::Array3f target = {100.0f, -300.0f, 200.0f};
        const auto new_center = tt::impl::computeParentCenter(node, target);
        CHECK(new_center[0] == doctest::Approx(1.f));
        CHECK(new_center[1] == doctest::Approx(-1.f));
        CHECK(new_center[2] == doctest::Approx(1.f));
    }
}

TEST_CASE("makeParentAndLinkNode") {
    {
        auto node = std::make_unique<tt::Node<Point<float>>>();
        node->center = {0.0f, 0.0f, 0.0f};
        node->half_extent = 1.0f;

        auto parent = tt::impl::makeParentAndLinkNode(std::move(node), {10.0f, 10.0f, 10.0f});
        CHECK(parent->center[0] == doctest::Approx(1.0f));
        CHECK(parent->center[1] == doctest::Approx(1.0f));
        CHECK(parent->center[2] == doctest::Approx(1.0f));
        CHECK(parent->half_extent == doctest::Approx(2.0f));

        auto count = std::count_if(parent->children.begin(), parent->children.end(),
                                   [](const auto &child) { return child != nullptr; });
        CHECK(count == 1);

        auto child =
            std::ranges::find_if(parent->children, [](const auto &c) { return c != nullptr; });
        CHECK(child != parent->children.end());
        CHECK((*child)->center[0] == doctest::Approx(0.0f));
        CHECK((*child)->center[1] == doctest::Approx(0.0f));
        CHECK((*child)->center[2] == doctest::Approx(0.0f));
        CHECK((*child)->half_extent == doctest::Approx(1.0f));
    }
    {
        auto node = std::make_unique<tt::Node<Point<float>>>();
        node->center = {1.0f, 1.0f, 1.0f};
        node->half_extent = 0.1f;

        auto parent = tt::impl::makeParentAndLinkNode(std::move(node), {-10.0f, -10.0f, -10.0f});
        CHECK(parent->center[0] == doctest::Approx(0.9f));
        CHECK(parent->center[1] == doctest::Approx(0.9f));
        CHECK(parent->center[2] == doctest::Approx(0.9f));
        CHECK(parent->half_extent == doctest::Approx(.2f));

        auto count = std::count_if(parent->children.begin(), parent->children.end(),
                                   [](const auto &child) { return child != nullptr; });
        CHECK(count == 1);

        auto child =
            std::ranges::find_if(parent->children, [](const auto &c) { return c != nullptr; });
        CHECK(child != parent->children.end());
        CHECK((*child)->center[0] == doctest::Approx(1.0f));
        CHECK((*child)->center[1] == doctest::Approx(1.0f));
        CHECK((*child)->center[2] == doctest::Approx(1.0f));
        CHECK((*child)->half_extent == doctest::Approx(.1f));
    }
}

TEST_CASE("growTreeUntilAbsorption") {
    {
        // Grow Positive
        auto node = std::make_unique<tt::Node<Point<float>>>();
        node->center = {0.0f, 0.0f, 0.0f};
        node->half_extent = 0.5f;

        // const auto rec = rerun::RecordingStream("growTreeUntilAbsorption");
        // rec.spawn().exit_on_failure();
        //
        // rec.set_time_sequence("frame", 0);
        // rec.log("boxes", toRerunBoxes(*node));

        tt::BoundingBox<float> bbox{{9.5f, 9.5f, 9.5f}, {10.5f, 10.5f, 10.0f}};
        auto root = tt::impl::growTreeUntilAbsorption(std::move(node), bbox);

        // rec.set_time_sequence("frame", 1);
        // rec.log("boxes", toRerunBoxes(bbox, rerun::Color(0, 255, 0)));
        // rec.set_time_sequence("frame", 2);
        // rec.log("boxes", toRerunBoxes(*root, rerun::Color(0, 255, 255)));

        CHECK(tt::impl::isBoxInNode(bbox, *root));
    }
    {
        // Grow negative
        auto node = std::make_unique<tt::Node<Point<float>>>();
        node->center = {1.0f, 2.0f, 3.0f};
        node->half_extent = 0.5f;

        tt::BoundingBox<float> bbox{{-6.f, -6.f, -6.f}, {-3.f, -4.f, -5.f}};
        auto root = tt::impl::growTreeUntilAbsorption(std::move(node), bbox);

        CHECK(tt::impl::isBoxInNode(bbox, *root));

        // const auto rec = rerun::RecordingStream("growTreeUntilAbsorption");
        // rec.spawn().exit_on_failure();
        // rec.set_time_sequence("frame", 0);
        // rec.log("boxes", toRerunBoxes(bbox, rerun::Color(0, 255, 0)));
        // rec.set_time_sequence("frame", 1);
        // rec.log("boxes", toRerunBoxes(*root, rerun::Color(0, 255, 255)));
    }
    {
        // Grow all directions
        auto node = std::make_unique<tt::Node<Point<float>>>();
        node->center = {1.0f, 2.0f, 3.0f};
        node->half_extent = 0.5f;

        tt::BoundingBox<float> bbox{{-100.f, -100.f, -100.f}, {100.f, 100.f, 100.f}};
        auto root = tt::impl::growTreeUntilAbsorption(std::move(node), bbox);
        CHECK(tt::impl::isBoxInNode(bbox, *root));
        //
        // const auto rec = rerun::RecordingStream("growTreeUntilAbsorption");
        // rec.spawn().exit_on_failure();
        // rec.set_time_sequence("frame", 0);
        // rec.log("boxes", toRerunBoxes(bbox, rerun::Color(0, 255, 0)));
        // rec.set_time_sequence("frame", 1);
        // rec.log("boxes", toRerunBoxes(*root, rerun::Color(0, 255, 255)));
    }
}

TEST_CASE("findClosestLeafNode") {
    constexpr float kMinExtent = 0.01f;
    {
        tt::TentacleTree<Point<float>> tree(2, kMinExtent);
        std::vector<Point<float>> points;
        points.push_back({0, 0, 0});
        tree.insert(points.begin(), points.end());

        auto &leaf = tt::impl::findClosestLeafNode(*tree.root(), {0.1f, 0.1f, 0.1f}, kMinExtent);

        // There can only be one leaf here and it's the one with the only point
        REQUIRE(leaf.points.size() == 1);
        for (const auto &c : leaf.children) {
            REQUIRE(c == nullptr);
        }
        REQUIRE(leaf.center[0] == doctest::Approx(0.0f));
        REQUIRE(leaf.center[1] == doctest::Approx(0.0f));
        REQUIRE(leaf.center[2] == doctest::Approx(0.0f));

        // If we change the query we should still get the same leaf because there's only one leaf
        auto &other_leaf =
            tt::impl::findClosestLeafNode(*tree.root(), {100.f, 222.1f, 444.1f}, kMinExtent);
        REQUIRE(other_leaf.points.size() == 1);
        for (const auto &c : leaf.children) {
            REQUIRE(c == nullptr);
        }
        REQUIRE(other_leaf.center[0] == doctest::Approx(0.0f));
        REQUIRE(other_leaf.center[1] == doctest::Approx(0.0f));
        REQUIRE(other_leaf.center[2] == doctest::Approx(0.0f));
    }
    {
        tt::TentacleTree<Point<float>> tree(2, kMinExtent);
        std::vector<Point<float>> points;
        points.push_back({5, 5, 5});
        tree.insert(points.begin(), points.end());

        const auto &leaf =
            tt::impl::findClosestLeafNode(*tree.root(), {-220.1f, -230.1f, 100.1f}, kMinExtent);

        // There can only be one leaf here and it's the one with the only point
        REQUIRE(leaf.points.size() == 1);
        for (const auto &c : leaf.children) {
            REQUIRE(c == nullptr);
        }
        REQUIRE(leaf.center[0] == doctest::Approx(5.0f));
        REQUIRE(leaf.center[1] == doctest::Approx(5.0f));
        REQUIRE(leaf.center[2] == doctest::Approx(5.0f));
    }
    {
        tt::TentacleTree<Point<float>> tree(2, kMinExtent);
        std::vector<Point<float>> points{{-10, -10, -10}, {10, 10, 10}};
        tree.insert(points.begin(), points.end());
        tree.insert(points.begin(), points.end());
        const auto &leaf =
            tt::impl::findClosestLeafNode(*tree.root(), {-1.0f, -1.0f, 0.0f}, kMinExtent);

        // const auto rec = rerun::RecordingStream("tentacle_tree_find_closest_leaf");
        // rec.spawn().exit_on_failure();
        // rec.log("points", rerun::Points3D(toRerunPositions(tree)));
        // rec.log("tree", toRerunBoxes(tree, rerun::Color(0, 0, 255)));

        REQUIRE(leaf.points.size() == 2);
        REQUIRE(leaf.points[0].x() == doctest::Approx(-10.0f));
        REQUIRE(leaf.points[0].y() == doctest::Approx(-10.0f));
        REQUIRE(leaf.points[0].z() == doctest::Approx(-10.0f));
    }
}

TEST_CASE_FIXTURE(PointCloudTestFixture<float>, "knnSearch") {
    using PointF = Point<float>;
    SUBCASE("All in first leaf") {
        tt::TentacleTree<PointF> tree(4, 0.01f);
        tree.insert(points_.begin(), points_.end());
        tree.insert(points_.begin(), points_.end());
        tree.insert(points_.begin(), points_.end());
        tree.insert(points_.begin(), points_.end());

        const PointF query_point{{10.0f, 10.0f, 10.0f}};
        constexpr std::size_t k = 4;
        auto neighbors = tree.knnSearch(query_point, k);
        REQUIRE(neighbors.size() == k);
        for (const auto &neighbor : neighbors) {
            REQUIRE(neighbor.point.get().x() == doctest::Approx(1.0f));
            REQUIRE(neighbor.point.get().y() == doctest::Approx(1.0f));
            REQUIRE(neighbor.point.get().z() == doctest::Approx(1.0f));
        }
    }
    SUBCASE("More points in leaf than k") {
        tt::TentacleTree<PointF> tree(4, 0.01f);
        tree.insert(points_.begin(), points_.end());
        tree.insert(points_.begin(), points_.end());
        tree.insert(points_.begin(), points_.end());
        tree.insert(points_.begin(), points_.end());

        const auto rec = rerun::RecordingStream("knnSearch");
        rec.spawn().exit_on_failure();
        rec.log("points", rerun::Points3D(toRerunPositions(points_)));
        rec.log("tree", toRerunBoxes(tree, rerun::Color(0, 0, 255)));

        const PointF query_point{{1.0f, 1.0f, 1.2f}};
        constexpr std::size_t k = 3;
        auto neighbors = tree.knnSearch(query_point, k);
        REQUIRE(neighbors.size() == k);
        for (const auto &neighbor : neighbors) {
            REQUIRE(neighbor.point.get().x() == doctest::Approx(1.0f));
            REQUIRE(neighbor.point.get().y() == doctest::Approx(1.0f));
            REQUIRE(neighbor.point.get().z() == doctest::Approx(1.0f));
        }
    }
}

TEST_CASE("BoundedPriorityQueue::push") {
    SUBCASE("Basic push when queue is not full (max-heap)") {
        tt::impl::BoundedPriorityQueue<int> pq(std::less<int>{}, 3);

        CHECK(pq.empty());
        CHECK(pq.size() == 0);
        CHECK_FALSE(pq.isFull());

        pq.push(5);
        CHECK(pq.size() == 1);
        CHECK(pq.top() == 5);
        CHECK_FALSE(pq.isFull());

        pq.push(3);
        CHECK(pq.size() == 2);
        CHECK(pq.top() == 5); // max-heap, so 5 is at top
        CHECK_FALSE(pq.isFull());

        pq.push(7);
        CHECK(pq.size() == 3);
        CHECK(pq.top() == 7); // 7 is now the largest
        CHECK(pq.isFull());
    }

    SUBCASE("Bounded behavior: pushing better element when full") {
        tt::impl::BoundedPriorityQueue<int> pq(std::less<int>{}, 3);

        pq.push(10);
        pq.push(20);
        pq.push(30);
        CHECK(pq.isFull());
        CHECK(pq.top() == 30); // largest element

        // Push a smaller (better) element - should replace 30
        pq.push(5);
        CHECK(pq.size() == 3);
        CHECK(pq.top() == 20); // 30 was removed, now 20 is largest
    }

    SUBCASE("Bounded behavior: pushing worse element when full") {
        tt::impl::BoundedPriorityQueue<int> pq(std::less<int>{}, 3);

        pq.push(10);
        pq.push(20);
        pq.push(30);
        CHECK(pq.top() == 30);

        // Push a larger (worse) element - should be rejected
        pq.push(40);
        CHECK(pq.size() == 3);
        CHECK(pq.top() == 30); // unchanged
    }

    SUBCASE("Min-heap behavior with std::greater") {
        tt::impl::BoundedPriorityQueue<double, std::greater<double>> pq(std::greater<double>{}, 3);

        pq.push(10.0);
        pq.push(5.0);
        pq.push(15.0);
        CHECK(pq.isFull());
        CHECK(pq.top() == doctest::Approx(5.0)); // min-heap, smallest at top

        // Push a larger (better for k-NN) element - should replace 5.0
        pq.push(20.0);
        CHECK(pq.size() == 3);
        CHECK(pq.top() == doctest::Approx(10.0)); // 5.0 was removed
    }

    SUBCASE("Custom comparator for pairs") {
        using DistIndexPair = std::pair<double, int>;
        auto comp = [](const DistIndexPair &a, const DistIndexPair &b) {
            return a.first < b.first; // max-heap by distance
        };

        tt::impl::BoundedPriorityQueue<DistIndexPair, decltype(comp)> pq(comp, 3);

        pq.push({1.5, 0});
        pq.push({2.5, 1});
        pq.push({0.5, 2});

        CHECK(pq.isFull());
        CHECK(pq.top().first == doctest::Approx(2.5)); // largest distance
        CHECK(pq.top().second == 1);

        // Push closer point - should replace furthest
        pq.push({0.3, 3});
        CHECK(pq.size() == 3);
        CHECK(pq.top().first == doctest::Approx(1.5)); // 2.5 was removed
    }

    SUBCASE("Edge case: max_size = 1") {
        tt::impl::BoundedPriorityQueue<int> pq(std::less<int>{}, 1);

        pq.push(10);
        CHECK(pq.isFull());
        CHECK(pq.top() == 10);

        pq.push(5); // better
        CHECK(pq.size() == 1);
        CHECK(pq.top() == 5);

        pq.push(20); // worse
        CHECK(pq.size() == 1);
        CHECK(pq.top() == 5); // unchanged
    }

    SUBCASE("Float values with proper comparison") {
        tt::impl::BoundedPriorityQueue<float> pq(std::less<float>{}, 2);

        pq.push(3.14f);
        pq.push(2.71f);
        CHECK(pq.isFull());
        CHECK(pq.top() == doctest::Approx(3.14f));

        pq.push(1.41f); // smaller, should replace 3.14
        CHECK(pq.size() == 2);
        CHECK(pq.top() == doctest::Approx(2.71f));
    }

    SUBCASE("Sequential pushes maintaining heap property") {
        tt::impl::BoundedPriorityQueue<int> pq(std::less<int>{}, 5);

        // Push elements in various orders
        pq.push(50);
        CHECK(pq.top() == 50);

        pq.push(30);
        CHECK(pq.top() == 50);

        pq.push(70);
        CHECK(pq.top() == 70);

        pq.push(20);
        CHECK(pq.top() == 70);

        pq.push(60);
        CHECK(pq.top() == 70);
        CHECK(pq.isFull());

        // Now it's full, push smaller values
        pq.push(10);
        CHECK(pq.top() == 60); // 70 removed

        pq.push(5);
        CHECK(pq.top() == 50); // 60 removed
    }

    SUBCASE("KnnResult with max-heap for k-NN search") {
        // Create test points with different coordinates
        std::vector<Point<float>> points = {{{1.0f, 2.0f, 3.0f}},
                                            {{4.0f, 5.0f, 6.0f}},
                                            {{7.0f, 8.0f, 9.0f}},
                                            {{0.5f, 0.5f, 0.5f}},
                                            {{10.0f, 10.0f, 10.0f}}};

        // Create KnnResults with different distances
        std::vector<tt::KnnResult<Point<float>>> results = {{std::ref(points[0]), 5.0f},
                                                            {std::ref(points[1]), 10.0f},
                                                            {std::ref(points[2]), 15.0f},
                                                            {std::ref(points[3]), 2.0f},
                                                            {std::ref(points[4]), 20.0f}};

        // For k-NN, we want to keep k closest points, so use max-heap
        // Top will be the furthest among the k closest
        tt::impl::BoundedPriorityQueue<tt::KnnResult<Point<float>>> pq(
            std::less<tt::KnnResult<Point<float>>>{}, 3);

        CHECK(pq.empty());

        // Push first 3 results
        pq.push(results[0]); // distance 5.0
        CHECK(pq.size() == 1);
        CHECK(pq.top().distance == doctest::Approx(5.0f));

        pq.push(results[1]); // distance 10.0
        CHECK(pq.size() == 2);
        CHECK(pq.top().distance == doctest::Approx(10.0f)); // max of (5, 10)

        pq.push(results[2]); // distance 15.0
        CHECK(pq.size() == 3);
        CHECK(pq.isFull());
        CHECK(pq.top().distance == doctest::Approx(15.0f)); // max of (5, 10, 15)

        // Now queue is full with distances: 5, 10, 15
        // Push a closer point (distance 2.0) - should replace furthest (15.0)
        pq.push(results[3]);
        CHECK(pq.size() == 3);
        CHECK(pq.top().distance == doctest::Approx(10.0f)); // max of (2, 5, 10)

        // Push a further point (distance 20.0) - should be rejected
        pq.push(results[4]);
        CHECK(pq.size() == 3);
        CHECK(pq.top().distance == doctest::Approx(10.0f)); // unchanged

        // Verify the point reference is correctly stored
        CHECK(&pq.top().point.get() == &points[1]); // Top is results[1] (distance 10.0)
    }
}

TEST_CASE("KNN search") {
    constexpr std::size_t kBucketSize = 5;
    constexpr float kMinNodeSize = 0.1f;
    tt::TentacleTree<Point<float>> tree(kBucketSize, kMinNodeSize);
    // clang-format off
    std::vector<Point<float>> points = {
        {10, 10, 10},
        {-10, -10, -10},
        {2, 2, 2},
        {2, 2, 2},
        {2, 2, 2},
        {-2, 2, 2},
        {2, 2, 2},
        {2, 2, 2},
    };
    // clang-format on
    tree.insert(points.begin(), points.end());

    // const auto rec = rerun::RecordingStream("tentacle_tree_find_closest_leaf");
    // rec.spawn().exit_on_failure();
    // rec.log("points", rerun::Points3D(toRerunPositions(tree)));
    // rec.log("tree", toRerunBoxes(tree, rerun::Color(0, 0, 255)));

    SUBCASE("Inside query - single") {
        const Point<float> point{0, 0, 0};
        auto neighbours = tree.knnSearch(point, 1);
        CHECK(neighbours.size() == 1);
        const auto &res = neighbours[0].point.get();
        CHECK(res.x() == doctest::Approx(2.0));
    }

    SUBCASE("Inside query - multiple") {}
}
