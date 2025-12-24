#include <tt/tentacle_tree.hpp>

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

template <typename FloatT>
struct Point {
    using ValueT = FloatT;
    FloatT x() const { return coords[0]; }
    FloatT y() const { return coords[1]; }
    FloatT z() const { return coords[2]; }
    FloatT operator[](std::size_t idx) const { return coords[idx]; }
    FloatT coords[3];

    Point<FloatT>& operator+=(const Point<FloatT> &other) {
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

        auto [min_coords, max_coords] =
            tt::impl::computeBoundingBox<float>(points.begin(), points.end());

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

        auto [min_coords, max_coords] =
            tt::impl::computeBoundingBox<double>(points.begin(), points.end());

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

        auto [min_coords, max_coords] =
            tt::impl::computeBoundingBox<int>(points.begin(), points.end());

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
        std::array<float, 3> min_coords = {-1.0f, -2.0f, 1.0f};
        std::array<float, 3> max_coords = {1.0f, 2.0f, 4.0f};

        auto [center, max_extent] =
            tt::impl::computeCenterAndMaxExtent<float>(min_coords, max_coords);
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
        auto [center, max_extent] =
            tt::impl::computeCenterAndMaxExtent<double>(min_coords, max_coords);

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
        auto code = tt::impl::computeMortonCode<Point<float>, float>(p0, center);
        CHECK(code == 0b0111);
        code = tt::impl::computeMortonCode<Point<float>, float>(p1, center);
        CHECK(code == 0b0110);
        code = tt::impl::computeMortonCode<Point<float>, float>(p2, center);
        CHECK(code == 0b0100);
        code = tt::impl::computeMortonCode<Point<float>, float>(p3, center);
        CHECK(code == 0b0101);
        code = tt::impl::computeMortonCode<Point<float>, float>(p4, center);
        CHECK(code == 0b0011);
        code = tt::impl::computeMortonCode<Point<float>, float>(p5, center);
        CHECK(code == 0b0010);
        code = tt::impl::computeMortonCode<Point<float>, float>(p6, center);
        CHECK(code == 0b0000);
        code = tt::impl::computeMortonCode<Point<float>, float>(p7, center);
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
        auto code = tt::impl::computeMortonCode<Point<float>, float>(p0, center);
        CHECK(code == 0b0111);
        code = tt::impl::computeMortonCode<Point<float>, float>(p1, center);
        CHECK(code == 0b0110);
        code = tt::impl::computeMortonCode<Point<float>, float>(p2, center);
        CHECK(code == 0b0100);
        code = tt::impl::computeMortonCode<Point<float>, float>(p3, center);
        CHECK(code == 0b0101);
        code = tt::impl::computeMortonCode<Point<float>, float>(p4, center);
        CHECK(code == 0b0011);
        code = tt::impl::computeMortonCode<Point<float>, float>(p5, center);
        CHECK(code == 0b0010);
        code = tt::impl::computeMortonCode<Point<float>, float>(p6, center);
        CHECK(code == 0b0000);
        code = tt::impl::computeMortonCode<Point<float>, float>(p7, center);
        CHECK(code == 0b0001);
    }
}