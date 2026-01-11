#ifndef TT_IMPL_INDEXED_VIEW_HPP
#define TT_IMPL_INDEXED_VIEW_HPP

#include <ranges>
#include <type_traits>
#include <vector>

template <std::ranges::random_access_range Container, std::ranges::random_access_range Index>
class IndexedView : public std::ranges::view_interface<IndexedView<Container, Index>> {
  public:
    using index_type = std::ranges::range_difference_t<Index>;
    using value_type = std::ranges::range_value_t<Container>;
    using container_view_t = std::ranges::views::all_t<Container>;
    using index_view_t = std::ranges::views::all_t<Index>;

    template <typename C, typename I>
    IndexedView(C &&container, I &&index)
        : container_(std::views::all(std::forward<C>(container))),
          index_(std::views::all(std::forward<I>(index))) {}

    template <typename Parent>
    struct iterator_impl {
        using index_iter_t = std::ranges::iterator_t<
            std::conditional_t<std::is_const_v<Parent>, const index_view_t, index_view_t>>;
        using container_t =
            std::conditional_t<std::is_const_v<Parent>, const container_view_t, container_view_t>;
        using difference_type = std::ranges::range_difference_t<Index>;
        using value_type = std::ranges::range_value_t<Container>;
        using reference =
            decltype(std::declval<
                     container_t &>()[std::declval<std::ranges::range_value_t<Index>>()]);
        using pointer = std::add_pointer_t<reference>;
        using iterator_category = std::random_access_iterator_tag;

        index_iter_t it_;
        Parent *parent_;

        iterator_impl() = default;
        iterator_impl(index_iter_t it, Parent *parent) : it_(it), parent_(parent) {}

        reference operator*() const {
            using diff_t = std::ranges::range_difference_t<container_t>;
            return (*parent_).container_[static_cast<diff_t>(*it_)];
        }
        pointer operator->() const {
            using diff_t = std::ranges::range_difference_t<container_t>;
            return &(*parent_).container_[static_cast<diff_t>(*it_)];
        }
        reference operator[](difference_type n) const {
            using diff_t = std::ranges::range_difference_t<container_t>;
            return (*parent_).container_[static_cast<diff_t>(*(it_ + n))];
        }

        iterator_impl &operator++() {
            ++it_;
            return *this;
        }
        iterator_impl operator++(int) {
            auto tmp = *this;
            ++(*this);
            return tmp;
        }
        iterator_impl &operator--() {
            --it_;
            return *this;
        }
        iterator_impl operator--(int) {
            auto tmp = *this;
            --(*this);
            return tmp;
        }
        iterator_impl &operator+=(difference_type n) {
            it_ += n;
            return *this;
        }
        iterator_impl &operator-=(difference_type n) {
            it_ -= n;
            return *this;
        }
        iterator_impl operator+(difference_type n) const { return iterator_impl(it_ + n, parent_); }
        iterator_impl operator-(difference_type n) const { return iterator_impl(it_ - n, parent_); }
        difference_type operator-(const iterator_impl &other) const { return it_ - other.it_; }

        bool operator==(const iterator_impl &other) const { return it_ == other.it_; }
        bool operator!=(const iterator_impl &other) const { return it_ != other.it_; }
        bool operator<(const iterator_impl &other) const { return it_ < other.it_; }
        bool operator>(const iterator_impl &other) const { return it_ > other.it_; }
        bool operator<=(const iterator_impl &other) const { return it_ <= other.it_; }
        bool operator>=(const iterator_impl &other) const { return it_ >= other.it_; }
    };

    using iterator = iterator_impl<IndexedView>;
    using const_iterator = iterator_impl<const IndexedView>;

    iterator begin() { return iterator(std::ranges::begin(index_), this); }
    iterator end() { return iterator(std::ranges::end(index_), this); }
    const_iterator begin() const { return const_iterator(std::ranges::begin(index_), this); }
    const_iterator end() const { return const_iterator(std::ranges::end(index_), this); }
    const_iterator cbegin() const { return begin(); }
    const_iterator cend() const { return end(); }

    index_type size() const { return static_cast<index_type>(std::ranges::size(index_)); }
    [[nodiscard]] bool empty() const { return std::ranges::empty(index_); }

    decltype(auto) operator[](index_type n) {
        using diff_t = std::ranges::range_difference_t<container_view_t>;
        return container_[static_cast<diff_t>(index_[n])];
    }
    decltype(auto) operator[](index_type n) const {
        using diff_t = std::ranges::range_difference_t<container_view_t>;
        return container_[static_cast<diff_t>(index_[n])];
    }

  private:
    container_view_t container_;
    index_view_t index_;
};

// Deduction guide using the all_t
// See https://medium.com/@simontoth/daily-bit-e-of-c-implementing-custom-views-bb21e63a2d4f
template <std::ranges::random_access_range Container, std::ranges::random_access_range Index>
IndexedView(Container &&container, Index &&index)
    -> IndexedView<std::ranges::views::all_t<Container>, std::ranges::views::all_t<Index>>;

#endif // TT_IMPL_INDEXED_VIEW_HPP
