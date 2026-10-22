//
// Copyright 2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_CONCURRENCY_PARALLEL_FOR_H
#define CODELIBRARY_CONCURRENCY_PARALLEL_FOR_H

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <type_traits>

#include "codelibrary/concurrency/thread_pool.h"

namespace cl {

/**
 * Modified from Ceres-solver.
 *
 * Parallel implementation through a fixed size thread pool with a shared task
 * queue.
 */
class ParallelInvoke {
    // Maximal number of work items scheduled for a single thread
    //  - Lower number of work items results in larger runtimes on unequal tasks
    //  - Higher number of work items results in larger losses for
    //    synchronization
    static constexpr int WORK_BLOCKS_PER_THREAD = 4;

    // Primary template for is_invocable
    template <typename, typename = void>
    struct IsInvocable : std::false_type {};

    template <typename F>
    struct IsInvocable<F, decltype(void(std::declval<F>()(std::declval<int>(),
                                                          std::declval<int>())),
                                   void())>
        : std::true_type {};

    /**
     * This class creates a thread safe barrier which will block until a
     * pre-specified number of threads call Finished. This allows us to block
     * the main thread until all the parallel threads are finished processing
     * all the work.
     */
    class BlockUntilFinished {
    public:
        explicit BlockUntilFinished(int n_total_jobs)
            : n_total_jobs_(n_total_jobs) {}

        /**
         * Increment the number of jobs that have been processed by the number
         * of jobs processed by caller and signal the blocking thread if all
         * jobs have finished.
         */
        void Finished(int n_jobs_finished) {
            if (n_jobs_finished == 0) return;

            std::lock_guard<std::mutex> lock(mutex_);
            n_total_jobs_finished_ += n_jobs_finished;
            CHECK(n_total_jobs_finished_ <= n_total_jobs_);

            if (n_total_jobs_finished_ == n_total_jobs_) {
                condition_.notify_one();
            }
        }

        /**
         * Block until receiving confirmation of all jobs being finished.
         */
        void Block() {
            std::unique_lock<std::mutex> lock(mutex_);
            condition_.wait(lock, [this]() {
                return n_total_jobs_finished_ == n_total_jobs_;
            });
        }

     private:
        std::mutex mutex_;
        std::condition_variable condition_;
        int n_total_jobs_finished_ = 0;
        const int n_total_jobs_ = 0;
    };

    /**
     * Shared state between the parallel tasks. Each thread will use this
     * information to get the next block of work to be performed.
     */
    struct State {
        /**
         * The entire range [start, end) is split into n_work_blocks contiguous
         * disjoint intervals (blocks), which are as equal as possible given
         * total index count and requested number of blocks.
         *
         * Those n_work_blocks blocks are then processed in parallel.
         *
         * Total number of integer indices in interval [start, end) is
         * end - start, and when splitting them into 'n_work_blocks' blocks
         * we can either
         *  - Split into equal blocks when (end - start) is divisible by
         *    n_work_blocks
         *  - Split into blocks with size difference at most 1:
         *     - Size of the smallest blocks is (end - start) / n_work_blocks
         *     - (end - start) % n_work_blocks will need to be 1 index larger
         *
         * Note that this splitting is optimal in the sense of maximal
         * difference between block sizes, since splitting into equal blocks is
         * possible if and only if number of indices is divisible by number of
         * blocks.
         */
        State(int start, int end, int n_work_blocks)
            : start(start),
              end(end),
              n_work_blocks(n_work_blocks),
              base_block_size((end - start) / n_work_blocks),
              n_base_p1_sized_blocks((end - start) % n_work_blocks),
              block_id(0),
              thread_id(0),
              block_until_finished(n_work_blocks) {}

        // The start and end index of the for loop.
        const int start = 0, end = 0;

        // The number of blocks that need to be processed.
        const int n_work_blocks = 0;

        // Size of the smallest block.
        const int base_block_size;

        // Number of blocks of size base_block_size + 1
        const int n_base_p1_sized_blocks;

        // The next block of work to be assigned to a worker. The parallel for
        // loop range is split into n_work_blocks blocks of work, with a single
        // block of work being of size
        //  - base_block_size + 1 for the first n_base_p1_sized_blocks blocks,
        //  - base_block_size for the rest of the blocks;
        // blocks of indices are contiguous and disjoint.
        std::atomic<int> block_id;

        // Provides a unique thread ID among all active threads.
        // We do not schedule more than n_threads threads via thread pool
        // and caller thread might steal one ID.
        std::atomic<int> thread_id;

        // Used to signal when all the work has been completed. Thread safe.
        BlockUntilFinished block_until_finished;
    };

public:
    static ParallelInvoke* GetInstance() {
        static ParallelInvoke loop;
        return &loop;
    }

    /**
     * The problem of executing the function for the interval of [start, end)
     * is broken up into at most n_threads * WORK_BLOCKS_PER_THREAD blocks (each
     * of size at least min_block_size) and added to the thread pool.
     *
     * To avoid deadlocks, the calling thread is allowed to steal work from the
     * worker (thread) pool.
     *
     * This is implemented via a shared state between the tasks. In order for
     * the calling thread or thread pool to get a block of work, it will query
     * the shared state for the next block of work to be done. If there is
     * nothing left, it will return. We will exit the ParallelFor call when all
     * of the work has been done, not when all of the tasks have been popped off
     * the task queue.
     *
     * A unique thread ID among all active tasks will be acquired once for each
     * block of work.  This avoids the significant performance penalty for
     * acquiring it on every iteration of the for loop. The thread ID is
     * guaranteed to be in [0, n_threads).
     */
    template <typename F>
    void InvokeOnRange(int start, int end, int n_threads, F&& function,
                       int min_block_size) {
        CHECK(start >= 0);
        CHECK(start < end);
        CHECK(n_threads > 0);
        CHECK(min_block_size > 0);

        // Interval [start, end) is being split into
        // num_threads * work_blocks_per_thread contiguous disjoint blocks.
        //
        // In order to avoid creating empty blocks of work, we need to limit
        // number of work blocks by a total number of indices.
        const int n_work_blocks = std::min((end - start) / min_block_size,
                                           n_threads * WORK_BLOCKS_PER_THREAD);

        // We use a std::shared_ptr because the main thread can finish all
        // the work before the tasks have been popped off the queue. So the
        // shared state needs to exist for the duration of all the tasks.
        auto shared_state = std::make_shared<State>(start, end, n_work_blocks);

        // A function which tries to schedule another task in the thread pool
        // and perform several chunks of work. Function expects itself as the
        // argument in order to schedule next task in the thread pool.
        auto task = [this, shared_state, n_threads, &function]
                    (auto& task_copy) {
            int n_jobs_finished = 0;
            const int thread_id = shared_state->thread_id.fetch_add(1);

            // In order to avoid dead-locks in nested parallel for loops, task()
            // will be invoked n_threads + 1 times:
            //  - n_threads times via enqueueing task into thread pool
            //  - one more time in the main thread.
            // Tasks enqueued to thread pool might take some time before
            // execution, and the last task being executed will be terminated
            // here in order to avoid having more than n_threads active threads.
            if (thread_id >= n_threads) return;

            const int n_work_blocks = shared_state->n_work_blocks;

            if (thread_id + 1 < n_threads &&
                shared_state->block_id < n_work_blocks) {
                // Add another thread to the thread pool.
                // Note we are taking the task as value so the copy of
                // shared_state shared pointer (captured by value at declaration
                // of task lambda-function) is copied and the ref count is
                // increased. This is to prevent it from being deleted when the
                // main thread finishes all the work and exits before the
                // threads finish.
                this->thread_pool_.AddTask([task_copy]() {
                    task_copy(task_copy);
                });
            }

            int start = shared_state->start;
            int base_block_size = shared_state->base_block_size;
            int n_base_p1_sized_blocks = shared_state->n_base_p1_sized_blocks;

            while (true) {
                // Get the next available chunk of work to be performed.
                // If there is no work, return.
                int block_id = shared_state->block_id.fetch_add(1);
                if (block_id >= n_work_blocks) {
                    break;
                }
                ++n_jobs_finished;

                // For-loop interval [start, end) was split into n_work_blocks,
                // with n_base_p1_sized_blocks of size base_block_size + 1 and
                // remaining n_work_blocks - n_base_p1_sized_blocks of size
                // base_block_size.
                //
                // Then, start index of the block #block_id is given by a total
                // length of preceding blocks:
                //  * Total length of preceding blocks of size
                //    base_block_size + 1:
                //    min(block_id, n_base_p1_sized_blocks) *
                //    (base_block_size + 1)
                //
                //  * Total length of preceding blocks of size base_block_size:
                //    (block_id - min(block_id, n_base_p1_sized_blocks)) *
                //    base_block_size
                //
                // Simplifying sum of those quantities yields a following
                // expression for start index of the block #block_id
                const int cur_start = start + block_id * base_block_size +
                            std::min(block_id, n_base_p1_sized_blocks);

                // First n_base_p1_sized_blocks have size base_block_size + 1.
                //
                // Note that it is guaranteed that all blocks are within
                // [start, end) interval
                const int cur_end = cur_start + base_block_size +
                        (block_id < n_base_p1_sized_blocks ? 1 : 0);
                // Perform each task in current block.
                this->InvokeOnSegment(cur_start, cur_end, function);
            }
            shared_state->block_until_finished.Finished(n_jobs_finished);
        };

        // Start scheduling threads and doing work. We might end up with less
        // threads scheduled than expected, if scheduling overhead is larger
        // than the amount of work to be done.
        task(task);

        // Wait until all tasks have finished.
        shared_state->block_until_finished.Block();
    }

    /**
     * InvokeOnSegment for accepting a single index.
     */
    template <typename F>
    typename std::enable_if<!IsInvocable<F>::value>::type
    InvokeOnSegment(int start, int end, F&& function) {
       for (int i = start; i != end; ++i) {
            function(i);
        }
    }

    /**
     * InvokeOnSegment for accepting a range consist of two integers.
     */
    template <typename F>
    typename std::enable_if<IsInvocable<F>::value>::type
    InvokeOnSegment(int start, int end, F&& function) {
        function(start, end);
    }

    /**
     * The max number of threads in the thread pool.
     */
    int max_threads() const {
        return thread_pool_.size();
    }

private:
    ParallelInvoke() = default;

    ThreadPool thread_pool_;
};

/**
 * Execute the function for every element in the range [start, end) with at
 * most n_threads.
 *
 * When distributing workload between threads, it is assumed that each loop
 * iteration takes approximately equal time to complete.
 *
 * The following function signatures are supported:
 *  - Functions accepting a single loop index:
 *     - [](int index) { ... }
 *  - Functions accepting a range of loop index:
 *     - [](int start, int end) { ... }
 */
template <typename F>
void ParallelFor(int start, int end, int n_threads, F&& function,
                 int min_block_size = 1) {
    CHECK(n_threads > 0);
    CHECK(min_block_size > 0);

    if (start >= end) {
        return;
    }

    auto invoke = ParallelInvoke::GetInstance();
    if (n_threads == 1 || end - start < min_block_size * 2) {
        invoke->InvokeOnSegment(start, end, std::forward<F>(function));
        return;
    }

    invoke->InvokeOnRange(start, end, n_threads, std::forward<F>(function),
                          min_block_size);
}
template <typename F>
void ParallelFor(int start, int end, F&& function, int min_block_size = 1) {
    ParallelFor(start, end, std::thread::hardware_concurrency(), function,
                min_block_size);
}

} // namespace cl

#endif // CODELIBRARY_CONCURRENCY_PARALLEL_FOR_H
