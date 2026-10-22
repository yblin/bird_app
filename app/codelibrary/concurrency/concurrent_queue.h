//
// Copyright 2024-2026 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_CONCURRENCY_CONCURRENT_QUEUE_H_
#define CODELIBRARY_CONCURRENCY_CONCURRENT_QUEUE_H_

#include <atomic>
#include <new>

#include "codelibrary/base/log.h"
#include "codelibrary/base/memory.h"

namespace cl {

#ifdef __cpp_lib_hardware_interference_size
    using std::hardware_constructive_interference_size;
    using std::hardware_destructive_interference_size;
#else
    // 64 bytes on x86-64 │ L1_CACHE_BYTES │ L1_CACHE_SHIFT │
    // __cacheline_aligned │ ...
    constexpr std::size_t hardware_constructive_interference_size = 64;
    constexpr std::size_t hardware_destructive_interference_size = 64;
#endif

/**
 * Lock-free bounded concurrent queue.
 *
 * The code is inspired from Erik Rigtorp's MPMCQueue.
 *
 * All operations except construction and destruction are thread safe.
 */
template <typename T>
class ConcurrentQueue {
    struct Slot {
        Slot() = default;

        ~Slot() {
            if (turn & 1) {
                Destroy();
            }
        }

        template <typename... Args>
        void Construct(Args&&...args) {
            new (&storage) T(std::forward<Args>(args)...);
        }

        void Destroy() noexcept {
            reinterpret_cast<T *>(&storage)->~T();
        }

        T&& Move() {
            return std::move(reinterpret_cast<T&&>(storage));
        }

        // Align to avoid false sharing between adjacent slots.
        alignas(hardware_destructive_interference_size)
        std::atomic<int> turn = {0};
        typename std::aligned_storage<sizeof(T), alignof(T)>::type storage;
    };

public:
    /**
     * Construct a new ConcurrentQueue holding items of type T with specific
     * capacity.
     */
    explicit ConcurrentQueue(int capacity)
        : capacity_(capacity) {
        CHECK(capacity > 0);
        CHECK(capacity + 1 < INT_MAX);

        // Allocate one extra slot to prevent false sharing on the last slot.
        slots_ = reinterpret_cast<Slot*>(
                    AlignedAllocate((capacity_ + 1) * sizeof(Slot),
                                    alignof(Slot)));

        // Allocators are not required to honor alignment for over-aligned types
        // (see http://eel.is/c++draft/allocator.requirements#10) so we verify
        // alignment here.
        CHECK(reinterpret_cast<size_t>(slots_) % alignof(Slot) == 0);

        for (int i = 0; i < capacity_; ++i) {
            new (&slots_[i]) Slot();
        }

        const size_t hdi_size = hardware_destructive_interference_size;
        static_assert(alignof(Slot) == hdi_size,
                      "Slot must be aligned to cache line boundary to prevent "
                      "false sharing");
        static_assert(sizeof(Slot) % hdi_size == 0,
                      "Slot size must be a multiple of cache line size to "
                      "prevent false sharing between adjacent slots");

        static_assert(sizeof(ConcurrentQueue) % hdi_size == 0,
                      "Queue size must be a multiple of cache line size to "
                      "prevent false sharing between adjacent queues");
        static_assert(offsetof(ConcurrentQueue, tail_) -
                      offsetof(ConcurrentQueue, head_) ==
                      static_cast<std::ptrdiff_t>(hdi_size),
                      "head and tail must be a cache line apart to prevent "
                      "false sharing");
    }

    ~ConcurrentQueue() {
        if (!slots_) return;

        for (int i = 0; i < capacity_; ++i) {
            slots_[i].~Slot();
        }
        AlignedDeallocate(slots_);
    }

    // non-copyable and non-movable
    ConcurrentQueue(const ConcurrentQueue&) = delete;
    ConcurrentQueue &operator=(const ConcurrentQueue&) = delete;

    /**
     * Enqueue an item using inplace construction. Blocks if queue is full.
     *
     * 1. Acquire next write ticket from head.
     * 2. Wait for our turn (2 * (ticket / capacity)) to write slot
     *    (ticket % capacity).
     * 3. Set turn = turn + 1 to inform the readers we are done writing.
     */
    template <typename... Args>
    void emplace(Args &&...args) {
        const auto head = head_.fetch_add(1);
        auto& slot = slots_[idx(head)];
        while (turn(head) * 2 != slot.turn.load(std::memory_order_acquire)) {
        }

        slot.Construct(std::forward<Args>(args)...);
        slot.turn.store(turn(head) * 2 + 1, std::memory_order_release);
    }

    /**
     * Try to enqueue an item using inplace construction.
     *
     * Returns true on success and false if queue is full.
     */
    template <typename... Args>
    bool try_emplace(Args&&...args) {
        auto head = head_.load(std::memory_order_acquire);
        for (;;) {
            auto &slot = slots_[idx(head)];
            if (turn(head) * 2 == slot.turn.load(std::memory_order_acquire)) {
                if (head_.compare_exchange_strong(head, head + 1)) {
                    slot.Construct(std::forward<Args>(args)...);
                    slot.turn.store(turn(head) * 2 + 1,
                                    std::memory_order_release);
                    return true;
                }
            } else {
                auto const prevHead = head;
                head = head_.load(std::memory_order_acquire);
                if (head == prevHead) {
                    return false;
                }
            }
        }
    }

    /**
     * Enqueue an item using copy construction. Blocks if queue is full.
     */
    void push(const T& v) {
        emplace(v);
    }

    /**
     * Enqueue an item using move construction. Blocks if queue is full.
     */
    template <typename P>
    void push(P&& v) {
        emplace(std::forward<P>(v));
    }

    /**
     * Try to enqueue an item using copy construction.
     * Return true on success and false if queue is full.
     */
    bool try_push(const T& v) {
        return try_emplace(v);
    }

    /**
     * Try to enqueue an item using move construction.
     * Return true on success and false if queue is full.
     */
    template <typename P>
    bool try_push(P&& v) noexcept {
        return try_emplace(std::forward<P>(v));
    }

    /**
     * Dequeue an item by copying or moving the item into v. Blocks if queue is
     * empty.
     *
     * 1. Acquire next read ticket from tail.
     * 2. Wait for our turn (2 * (ticket / capacity) + 1) to read slot
     *    (ticket % capacity).
     * 3. Set turn = turn + 1 to inform the writers we are done reading.
     */
    void pop(T* v) {
        CHECK(v);

        const int tail = tail_.fetch_add(1);
        auto& slot = slots_[idx(tail)];
        while (turn(tail) * 2 + 1 != slot.turn.load(std::memory_order_acquire));
        *v = std::move(slot.Move());
        slot.Destroy();
        slot.turn.store(turn(tail) * 2 + 2, std::memory_order_release);
    }

    /**
     * Try to dequeue an item by copying or moving the item into v.
     * Return true on sucess and false if the queue is empty.
     */
    bool try_pop(T* v) {
        CHECK(v);

        auto tail = tail_.load(std::memory_order_acquire);
        for (;;) {
            auto& slot = slots_[idx(tail)];
            if (turn(tail) * 2 + 1 ==
                slot.turn.load(std::memory_order_acquire)) {
                if (tail_.compare_exchange_strong(tail, tail + 1)) {
                    *v = std::move(slot.Move());
                    slot.Destroy();
                    slot.turn.store(turn(tail) * 2 + 2,
                                    std::memory_order_release);
                    return true;
                }
            } else {
                const auto prev_tail = tail;
                tail = tail_.load(std::memory_order_acquire);
                if (tail == prev_tail) {
                    return false;
                }
            }
        }
    }

    /**
     * Returns the [approximate] number of elements in the queue.
     *
     * The size can be negative when the queue is empty and there is at least
     * one reader waiting. Since this is a concurrent queue the size is only a
     * best effort guess until all reader and writer threads have been joined.
     */
    int size() const {
        return head_.load(std::memory_order_relaxed) -
               tail_.load(std::memory_order_relaxed);
    }

    /**
     * Returns true if the queue is empty.
     *
     * Since this is a concurrent queue this is only a best effort guess until
     * all reader and writer threads have been joined.
     */
    bool empty() const {
        return size() <= 0;
    }

  private:
    constexpr int idx(int i) const {
        return static_cast<int>(static_cast<unsigned int>(i) %
                                static_cast<unsigned int>(capacity_));
    }

    constexpr int turn(int i) const {
        return static_cast<int>(static_cast<unsigned int>(i) /
                                static_cast<unsigned int>(capacity_));
     }

  private:
    const int capacity_ = 0;
    Slot* slots_ = nullptr;

    // Align to avoid false sharing between head_ and tail_.
    alignas(hardware_destructive_interference_size) std::atomic<int> head_ = 0;
    alignas(hardware_destructive_interference_size) std::atomic<int> tail_ = 0;
};

} // namespace cl

#endif // CODELIBRARY_CONCURRENCY_CONCURRENT_QUEUE_H_
