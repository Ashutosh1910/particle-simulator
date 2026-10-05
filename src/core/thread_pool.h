#pragma once
#include <condition_variable>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

// Small persistent thread pool with a static-partition parallel_for.
// The calling thread always takes part as worker 0, so threadCount()==1 means
// "run everything inline". Work is split into threadCount() contiguous chunks
// in index order, which keeps results that are concatenated per worker in the
// same order no matter how many threads are used.
// parallelFor is not re-entrant: do not call it from inside a job.
class ThreadPool {
public:
    using Job = std::function<void(int worker, int begin, int end)>;

    explicit ThreadPool(int threads = 1);
    ~ThreadPool();
    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    void setThreadCount(int threads);
    int threadCount() const { return threads_; }
    static int hardwareThreads();

    // Runs fn over [0, n). Ranges smaller than minPerThread*2 run inline on worker 0.
    void parallelFor(int n, const Job& fn, int minPerThread = 64);

private:
    void start(int threads);
    void stop();
    void workerLoop(int index, unsigned long long startGeneration);

    std::vector<std::thread> workers_;
    std::mutex mutex_;
    std::condition_variable wake_;
    std::condition_variable done_;
    const Job* job_ = nullptr;
    int jobSize_ = 0;
    int chunks_ = 1;
    int pending_ = 0;
    unsigned long long generation_ = 0;
    bool stopping_ = false;
    int threads_ = 1;
};
