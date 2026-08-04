#include "shape_match/pipeline_v3/WorkerPoolV3.h"

#include <algorithm>

namespace ShapeMatch {

thread_local WorkerPoolV3* WorkerPoolV3::activePool_ = nullptr;

WorkerPoolV3& WorkerPoolV3::shared()
{
    static WorkerPoolV3 pool(4);
    return pool;
}

void WorkerPoolV3::initializeShared()
{
    (void)shared();
}

WorkerPoolV3::WorkerPoolV3(int threadCount)
{
    threadCount = std::clamp(threadCount, 1, 4);
    workers_.reserve(static_cast<size_t>(threadCount));
    for (int index = 0; index < threadCount; ++index)
        workers_.emplace_back([this] { workerLoop(); });
}

WorkerPoolV3::~WorkerPoolV3()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
    }
    ready_.notify_all();
    for (std::thread& worker : workers_) worker.join();
}

bool WorkerPoolV3::runOnePendingTask()
{
    std::function<void()> task;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (tasks_.empty()) return false;
        task = std::move(tasks_.front());
        tasks_.pop_front();
    }
    task();
    return true;
}

void WorkerPoolV3::workerLoop()
{
    activePool_ = this;
    for (;;) {
        std::function<void()> task;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            ready_.wait(lock, [this] { return stopping_ || !tasks_.empty(); });
            if (stopping_ && tasks_.empty()) {
                activePool_ = nullptr;
                return;
            }
            task = std::move(tasks_.front());
            tasks_.pop_front();
        }
        task();
    }
}

} // namespace ShapeMatch
