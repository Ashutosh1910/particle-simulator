#include "thread_pool.h"

#include <algorithm>

ThreadPool::ThreadPool(int threads) { start(threads); }

ThreadPool::~ThreadPool() { stop(); }

int ThreadPool::hardwareThreads() {
    unsigned n = std::thread::hardware_concurrency();
    return n == 0 ? 1 : (int)n;
}

void ThreadPool::setThreadCount(int threads) {
    threads = std::clamp(threads, 1, 64);
    if (threads == threads_) return;
    stop();
    start(threads);
}

void ThreadPool::start(int threads) {
    threads_ = std::clamp(threads, 1, 64);
    stopping_ = false;
    // Workers start from the current generation (read here, before any job can be
    // posted) so they never miss or replay a job.
    unsigned long long gen = generation_;
    for (int i = 1; i < threads_; i++) workers_.emplace_back([this, i, gen] { workerLoop(i, gen); });
}

void ThreadPool::stop() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
    }
    wake_.notify_all();
    for (auto& t : workers_) t.join();
    workers_.clear();
}

void ThreadPool::workerLoop(int index, unsigned long long startGeneration) {
    unsigned long long seen = startGeneration;
    for (;;) {
        std::unique_lock<std::mutex> lock(mutex_);
        wake_.wait(lock, [&] { return stopping_ || generation_ != seen; });
        if (stopping_) return;
        seen = generation_;
        const Job* job = job_;
        int n = jobSize_, chunks = chunks_;
        lock.unlock();

        int begin = (int)((long long)n * index / chunks);
        int end = (int)((long long)n * (index + 1) / chunks);
        if (begin < end) (*job)(index, begin, end);

        lock.lock();
        if (--pending_ == 0) done_.notify_one();
    }
}

void ThreadPool::parallelFor(int n, const Job& fn, int minPerThread) {
    if (n <= 0) return;
    if (threads_ == 1 || n < minPerThread * 2) {
        fn(0, 0, n);
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        job_ = &fn;
        jobSize_ = n;
        chunks_ = threads_;
        pending_ = threads_ - 1;
        generation_++;
    }
    wake_.notify_all();
    int end = (int)((long long)n / threads_);
    if (end > 0) fn(0, 0, end);
    std::unique_lock<std::mutex> lock(mutex_);
    done_.wait(lock, [&] { return pending_ == 0; });
    job_ = nullptr;
}
