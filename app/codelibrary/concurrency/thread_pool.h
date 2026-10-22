//
// Copyright 2024 Yangbin Lin. All Rights Reserved.
//
// This file is part of the Code Library.
//

#ifndef CODELIBRARY_CONCURRENCY_THREAD_POOL_H_
#define CODELIBRARY_CONCURRENCY_THREAD_POOL_H_

#include <condition_variable>
#include <functional>
#include <future>
#include <mutex>
#include <queue>
#include <thread>

#include "codelibrary/base/array.h"

namespace cl {

/**
 * A thread-safe thread pool with an unbounded task queue and a fixed number of
 * workers (threads).
 *
 * The ThreadPool has two states:
 *
 * (1) Tasks may be added to the thread pool and will be executed as soon as a
 *     worker (thread) is available.
 * (2) The thread pool is destructing. The thread pool will signal all the
 *     workers (threads) to stop. The workers will finish all of the tasks that
 *     have already been added to the thread pool.
 */
class ThreadPool {
    template <typename F, typename... Args>
    using ReturnType =
#if defined(__cpp_lib_is_invocable) && __cpp_lib_is_invocable >= 201703L
        typename std::invoke_result<F&&, Args&&...>::type;
#else
        typename std::result_of<F&& (Args&&...)>::type;
#endif

public:
    ThreadPool()
        : ThreadPool(std::thread::hardware_concurrency()) {}

    explicit ThreadPool(int n_threads) {
        int max_threads = static_cast<int>(std::thread::hardware_concurrency());
        n_threads = std::min(max_threads, n_threads);
        n_threads = std::max(1, n_threads);

        for (int i = 0; i < n_threads; ++i) {
            threads_.emplace_back(&ThreadPool::ThreadMainLoop, this);
        }
    }

    ~ThreadPool() {
        Stop();
        for (std::thread& thread : threads_) {
            thread.join();
        }
    }

    /**
     * Add a new task to the pool.
     *
     * Usage:
     *   auto result = pool.AddTask([](int answer) { return answer; }, 42);
     *   // Get result from future
     *   LOG(INFO) << result.get();
     */
    template <class F, class... Args>
    auto AddTask(F&& f, Args&&... args) -> std::future<ReturnType<F, Args...>> {
        std::unique_lock<std::mutex> lock(mutex_);

        // Don't allow enqueueing after stopping the pool.
        CHECK(!stop_) << "AddTask() on stopped ThreadPool!";

        using R = ReturnType<F, Args...>;
        auto task = std::make_shared<std::packaged_task<R()>>(
                std::bind(std::forward<F>(f), std::forward<Args>(args)...));

        std::future<R> res = task->get_future();
        tasks_.emplace([task](){ (*task)(); });
        work_pending_condition_.notify_one();
        return res;
    }

    /**
     * Return the size of the thread pool.
     */
    int size() const {
        return threads_.size();
    }

private:
    /**
     * Main loop for the threads which blocks on the task queue until work
     * becomes available.
     * It will return if and only if Stop has been called.
     */
    void ThreadMainLoop() {
        while (true) {
            std::function<void()> task;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                work_pending_condition_.wait(lock, [this]{
                    return stop_ || !tasks_.empty();
                });

                if (stop_ && tasks_.empty()) return;
                task = std::move(tasks_.front());
                tasks_.pop();
            }

            task();
        }
    }

    /**
     * Signal all the threads to stop. It does not block until the threads are
     * finished.
     */
    void Stop() {
        std::unique_lock<std::mutex> lock(mutex_);
        stop_ = true;
        work_pending_condition_.notify_all();
    }

    // Need to keep track of threads so we can join them.
    Array<std::thread> threads_;

    // The task queue.
    std::queue<std::function<void()>> tasks_;

    std::mutex mutex_;
    std::condition_variable work_pending_condition_;
    bool stop_ = false;
};

} // namespace cl

#endif // CODELIBRARY_CONCURRENCY_THREAD_POOL_H_
