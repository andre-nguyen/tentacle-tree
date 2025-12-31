#ifndef TT_IMPL_BOUNDED_PRIORITY_QUEUE_HPP
#define TT_IMPL_BOUNDED_PRIORITY_QUEUE_HPP

#include <algorithm>
#include <vector>

namespace tt::impl {

template <typename T, typename Compare = std::less<T>>
class BoundedPriorityQueue {
  public:
    BoundedPriorityQueue(Compare comp, std::size_t max_size)
        : heap_{}, max_size_(max_size), comp_(comp) {}

    void push(const T &value);
    void pop();
    const T &top() const;
    [[nodiscard]] bool isFull() const;
    [[nodiscard]] std::size_t size() const;
    [[nodiscard]] bool empty() const;

    // Move out the internal heap vector
    [[nodiscard]] std::vector<T> &&destructiveGet() { return std::move(heap_); }

  private:
    std::vector<T> heap_;
    std::size_t max_size_;
    Compare comp_;
};

template <typename T, typename Compare>
void BoundedPriorityQueue<T, Compare>::push(const T &value) {
    if (heap_.size() < max_size_) {
        heap_.push_back(value);
        std::push_heap(heap_.begin(), heap_.end(), comp_);
    } else if (!heap_.empty() && comp_(value, heap_.front())) {
        // If the queue is full and the new value is better (smaller) than the worst (top),
        // replace the worst element
        std::pop_heap(heap_.begin(), heap_.end(), comp_);
        heap_.back() = value;
        std::push_heap(heap_.begin(), heap_.end(), comp_);
    }
}

template <typename T, typename Compare>
void BoundedPriorityQueue<T, Compare>::pop() {
    std::pop_heap(heap_.begin(), heap_.end(), comp_);
    heap_.pop_back();
}

template <typename T, typename Compare>
const T &BoundedPriorityQueue<T, Compare>::top() const {
    return heap_.front();
}

template <typename T, typename Compare>
bool BoundedPriorityQueue<T, Compare>::isFull() const {
    return heap_.size() >= max_size_;
}

template <typename T, typename Compare>
std::size_t BoundedPriorityQueue<T, Compare>::size() const {
    return heap_.size();
}

template <typename T, typename Compare>
bool BoundedPriorityQueue<T, Compare>::empty() const {
    return heap_.empty();
}

} // namespace tt::impl

#endif // TT_IMPL_BOUNDED_PRIORITY_QUEUE_HPP
