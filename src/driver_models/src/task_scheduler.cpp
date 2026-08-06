#include "driver_models/task_scheduler.h"
#include <ros/ros.h>

TaskScheduler::TaskScheduler(size_t thread_num)
    : stop_(false), started_(false), thread_num_(thread_num)
{
}

TaskScheduler::~TaskScheduler() {
    {
        std::lock_guard<std::mutex> lock(mtx_);
        stop_ = true;
    }
    cv_.notify_all();
    for (auto& t : workers_) t.join();
}

void TaskScheduler::addTask(const Task& task) {
    std::lock_guard<std::mutex> lock(mtx_);
    tasks_.push(task);
    cv_.notify_one();
}


void TaskScheduler::start() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (started_) return;

    started_ = true;

    for (size_t i = 0; i < thread_num_; ++i) {
        workers_.emplace_back([this]() { this->workerLoop(); });
    }

    cv_.notify_all();
}

void TaskScheduler::workerLoop() {
    std::unique_lock<std::mutex> lock(mtx_);

    // ⭐ 等待 start() 触发
    cv_.wait(lock, [this]() {
        return started_ || stop_;
    });

    ROS_WARN("Worker thread started.");

    while (!stop_) {

        if (tasks_.empty()) {
            cv_.wait(lock);
            continue;
        }

        auto now = std::chrono::steady_clock::now();
        auto task = tasks_.top();

        if (task.next_deadline > now) {
            cv_.wait_until(lock, task.next_deadline);
            continue;
        }

        tasks_.pop();
        lock.unlock();

        // 执行任务
        task.func();

        task.next_deadline += task.period;
        addTask(task);

        lock.lock();
    }
}

