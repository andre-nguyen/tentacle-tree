#ifndef TESTS_POINT_H
#define TESTS_POINT_H

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

#endif // TESTS_POINT_H
