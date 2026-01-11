#include <doctest/doctest.h>
#include <ranges>

#include "tt/impl/indexed_view.hpp"

TEST_CASE("IndexedView") {
    SUBCASE("constructor") {
        std::vector<int> container{10, 20, 30, 40};
        std::vector<size_t> index{2, 0, 3};
        IndexedView view(container, std::move(index));
        CHECK(view.size() == 3);
        CHECK(view[0] == 30);
        CHECK(view[1] == 10);
        CHECK(view[2] == 40);
    }
    SUBCASE("multiple views") {
        std::vector<int> container{10, 20, 30, 40};
        std::vector<size_t> index1{1, 2};
        std::vector<size_t> index2{3, 0};
        IndexedView view1(container, index1);
        IndexedView view2(container, index2);
        CHECK(view1.size() == 2);
        CHECK(view2.size() == 2);
        CHECK(view1[0] == 20);
        CHECK(view1[1] == 30);
        CHECK(view2[0] == 40);
        CHECK(view2[1] == 10);
    }
    SUBCASE("iterator") {
        std::vector<int> container{10, 20, 30, 40};
        std::vector<size_t> index{2, 0, 3};
        IndexedView view(container, index);
        std::vector<int> result;
        for (int &it : view) {
            result.push_back(it);
        }
        CHECK(result == std::vector<int>({30, 10, 40}));
        // Also test range-based for
        std::vector<int> result2;
        for (auto v : view)
            result2.push_back(v);
        CHECK(result2 == std::vector<int>({30, 10, 40}));
    }
    SUBCASE("composable with standard views") {
        std::vector<int> container{10, 20, 30, 40};
        std::vector<size_t> index{2, 0, 3};
        IndexedView view(container, index);
        // Combine with std::views::reverse
        std::vector<int> reversed;
        for (int v : view | std::views::reverse) {
            reversed.push_back(v);
        }
        CHECK(reversed == std::vector<int>({40, 10, 30}));
        // Combine with std::views::transform
        std::vector<int> doubled;
        for (int v : view | std::views::transform([](int x) { return x * 2; })) {
            doubled.push_back(v);
        }
        CHECK(doubled == std::vector<int>({60, 20, 80}));
    }
    SUBCASE("const correctness") {
        std::vector<int> container{10, 20, 30, 40};
        std::vector<size_t> index{2, 0, 3};
        const IndexedView view(container, index);
        CHECK(view.size() == 3);
        CHECK(view[0] == 30);
        std::vector<int> result;
        for (auto v : view)
            result.push_back(v);
        CHECK(result == std::vector<int>({30, 10, 40}));
    }
    SUBCASE("empty view") {
        std::vector<int> container;
        std::vector<size_t> index;
        IndexedView view(container, index);
        CHECK(view.size() == 0);
        CHECK(view.empty());
        std::vector<int> result;
        for (auto v : view)
            result.push_back(v);
        CHECK(result.empty());
    }
    SUBCASE("non-owning container (view as input)") {
        std::vector<int> container{10, 20, 30, 40};
        auto taken = container | std::views::take(3); // preserves random access
        std::vector<size_t> index{0, 2};
        IndexedView view(taken, index);
        std::vector<int> result;
        for (auto v : view)
            result.push_back(v);
        CHECK(result == std::vector<int>({10, 30}));
    }
    SUBCASE("modification through view") {
        std::vector<int> container{10, 20, 30, 40};
        std::vector<size_t> index{1, 2};
        IndexedView view(container, index);
        view[0] = 99;
        CHECK(container[1] == 99);
    }
    SUBCASE("random access iterator operations") {
        std::vector<int> container{10, 20, 30, 40};
        std::vector<size_t> index{2, 0, 3};
        IndexedView view(container, index);
        auto it = view.begin();
        CHECK(*it == 30);
        it += 2;
        CHECK(*it == 40);
        CHECK(it - view.begin() == 2);
        CHECK(view.begin() < it);
        CHECK(view.end() - view.begin() == 3);
        CHECK(view.begin()[1] == 10);
    }
    SUBCASE("size and empty") {
        std::vector<int> container{10, 20};
        std::vector<size_t> index{1};
        IndexedView view(container, index);
        CHECK(!view.empty());
        CHECK(view.size() == 1);
    }
    SUBCASE("copy and move semantics") {
        std::vector<int> container{10, 20, 30};
        std::vector<size_t> index{1, 2};
        IndexedView view1(container, index);
        IndexedView view2 = view1;
        CHECK(view2[0] == 20);
        IndexedView view3 = std::move(view1);
        CHECK(view3[1] == 30);
    }
    SUBCASE("const iteration") {
        std::vector<int> container{10, 20, 30};
        std::vector<size_t> index{2, 1};
        const IndexedView view(container, index);
        std::vector<int> result;
        for (int &it : view) {
            result.push_back(it);
        }
        CHECK(result == std::vector<int>({30, 20}));
    }
    SUBCASE("works with custom struct type") {
        struct Point {
            int x, y;
            bool operator==(const Point &other) const { return x == other.x && y == other.y; }
        };
        std::vector<Point> points{{1, 2}, {3, 4}, {5, 6}};
        std::vector<size_t> index{2, 0};
        IndexedView view(points, index);
        CHECK(view.size() == 2);
        CHECK(view[0] == Point{5, 6});
        CHECK(view[1] == Point{1, 2});
        std::vector<Point> result;
        for (const auto &p : view)
            result.push_back(p);
        CHECK(result == std::vector<Point>({{5, 6}, {1, 2}}));
        // Test modification
        view[1].x = 7;
        CHECK(points[0].x == 7);
    }
}
