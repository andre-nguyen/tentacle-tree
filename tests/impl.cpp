#include <doctest/doctest.h>
#include <tt/impl_tentacle_tree.hpp>

#include "point.h"

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

TEST_CASE("isNodeInSphere") {
    using tt::Node;
    using tt::impl::isNodeInSphere;
    using Point = Point<float>;

    SUBCASE("Node fully inside sphere") {
        Node<Point> node;
        node.center = {0.0f, 0.0f, 0.0f};
        node.half_extent = 1.0f;
        Point sphere_center = {0.0f, 0.0f, 0.0f};
        float sphere_radius = 5.0f;
        CHECK(isNodeInSphere(node, sphere_center, sphere_radius));
    }

    SUBCASE("Node partially inside sphere") {
        Node<Point> node;
        node.center = {0.0f, 0.0f, 0.0f};
        node.half_extent = 2.0f;
        Point sphere_center = {0.0f, 0.0f, 0.0f};
        float sphere_radius = 2.5f;
        CHECK_FALSE(isNodeInSphere(node, sphere_center, sphere_radius));
    }

    SUBCASE("Node completely outside sphere") {
        Node<Point> node;
        node.center = {10.0f, 10.0f, 10.0f};
        node.half_extent = 1.0f;
        Point sphere_center = {0.0f, 0.0f, 0.0f};
        float sphere_radius = 5.0f;
        CHECK_FALSE(isNodeInSphere(node, sphere_center, sphere_radius));
    }

    SUBCASE("Node exactly inscribed in sphere") {
        Node<Point> node;
        node.center = {0.0f, 0.0f, 0.0f};
        node.half_extent = 1.0f;
        Point sphere_center = {0.0f, 0.0f, 0.0f};
        // Distance from center to corner is sqrt(3) * half_extent
        float sphere_radius = std::sqrt(3.0f) * 1.0f;
        CHECK(isNodeInSphere(node, sphere_center, sphere_radius));
    }

    SUBCASE("Node at offset, sphere just large enough") {
        Node<Point> node;
        node.center = {2.0f, 0.0f, 0.0f};
        node.half_extent = 1.0f;
        Point sphere_center = {0.0f, 0.0f, 0.0f};
        // Farthest corner is at (3,1,1), distance = sqrt(3^2+1^2+1^2) = sqrt(11)
        float sphere_radius = std::sqrt(11.0f);
        CHECK(isNodeInSphere(node, sphere_center, sphere_radius));
        // Slightly smaller radius should fail
        CHECK_FALSE(isNodeInSphere(node, sphere_center, sphere_radius - 0.01f));
    }
}
