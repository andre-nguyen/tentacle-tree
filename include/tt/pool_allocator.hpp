#pragma once

#include <cstddef>
#include <memory>

#include <boost/pool/pool_alloc.hpp>

namespace tt {

// Custom allocator that uses a preallocated singleton pool
// The pool is preallocated on first use with a large chunk
template <typename T>
class preallocated_pool_allocator {
  public:
    using value_type = T;
    using size_type = std::size_t;
    using difference_type = std::ptrdiff_t;

    using pool_type = boost::singleton_pool<boost::fast_pool_allocator_tag, sizeof(T)>;

    preallocated_pool_allocator() noexcept {
        // Preallocate a large chunk: singleton_pool will manage this internally
        // Just accessing the pool triggers preallocation on first use
        (void)pool_type::malloc();
    }

    preallocated_pool_allocator(const preallocated_pool_allocator &) noexcept = default;

    template <typename U>
    preallocated_pool_allocator(const preallocated_pool_allocator<U> &) noexcept {}

    T *allocate(std::size_t n) { return static_cast<T *>(pool_type::malloc()); }

    void deallocate(T *p, std::size_t) noexcept { pool_type::free(p); }

    template <typename U>
    struct rebind {
        using other = preallocated_pool_allocator<U>;
    };

    bool operator==(const preallocated_pool_allocator &) const noexcept { return true; }
    bool operator!=(const preallocated_pool_allocator &) const noexcept { return false; }
};

} // namespace tt
