#pragma once

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <future>
#include <functional>
#include <mutex>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

namespace ShapeMatch {

class WorkerPoolV3
{
public:
    explicit WorkerPoolV3(int threadCount = 4);
    ~WorkerPoolV3();

    static WorkerPoolV3& shared();
    static void initializeShared();

    WorkerPoolV3(const WorkerPoolV3&) = delete;
    WorkerPoolV3& operator=(const WorkerPoolV3&) = delete;

    int threadCount() const noexcept { return static_cast<int>(workers_.size()); }

    template<class Function>
    auto submit(Function&& function) -> std::future<std::invoke_result_t<Function>>
    {
        using Result = std::invoke_result_t<Function>;
        auto task = std::make_shared<std::packaged_task<Result()>>(
            std::forward<Function>(function));
        std::future<Result> future = task->get_future();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            tasks_.emplace_back([task] { (*task)(); });
        }
        ready_.notify_one();
        return future;
    }

    template<class Result>
    Result wait(std::future<Result>& future)
    {
        waitUntilReady(future);
        return future.get();
    }

    void wait(std::future<void>& future)
    {
        waitUntilReady(future);
        future.get();
    }

    template<class Function>
    void parallelFor(int begin, int end, int requestedTasks, Function&& function)
    {
        if (end <= begin) return;
        const int taskCount = std::clamp(requestedTasks, 1, std::min(threadCount(), end - begin));
        if (taskCount == 1) {
            function(begin, end, 0);
            return;
        }
        std::vector<std::future<void>> futures;
        futures.reserve(static_cast<size_t>(taskCount));
        for (int taskIndex = 0; taskIndex < taskCount; ++taskIndex) {
            const int taskBegin = begin + (end - begin) * taskIndex / taskCount;
            const int taskEnd = begin + (end - begin) * (taskIndex + 1) / taskCount;
            futures.push_back(submit([&, taskBegin, taskEnd, taskIndex] {
                function(taskBegin, taskEnd, taskIndex);
            }));
        }
        for (std::future<void>& future : futures) wait(future);
    }

private:
    template<class Result>
    void waitUntilReady(std::future<Result>& future)
    {
        using namespace std::chrono_literals;
        while (future.wait_for(0ms) != std::future_status::ready) {
            if (activePool_ == this) {
                if (!runOnePendingTask()) future.wait_for(1ms);
            } else {
                future.wait();
            }
        }
    }

    bool runOnePendingTask();
    void workerLoop();

    static thread_local WorkerPoolV3* activePool_;

    std::mutex mutex_;
    std::condition_variable ready_;
    std::deque<std::function<void()>> tasks_;
    std::vector<std::thread> workers_;
    bool stopping_ = false;
};

} // namespace ShapeMatch
