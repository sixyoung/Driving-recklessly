#pragma once
#include <functional>
#include <chrono>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>

struct Task {
    std::function<void()> func;
    std::chrono::steady_clock::time_point next_deadline;
    std::chrono::milliseconds period;
    int vehicle_id;
    bool operator>(const Task& other) const {
        return next_deadline > other.next_deadline;
    }
};

class TaskScheduler {
public:
    TaskScheduler(size_t thread_num = std::thread::hardware_concurrency());
    ~TaskScheduler();
    void addTask(const Task& task);
    void start();

private:
    void workerLoop();

    std::priority_queue<Task, std::vector<Task>, std::greater<Task>> tasks_;
    std::vector<std::thread> workers_;
    std::mutex mtx_;
    std::condition_variable cv_;
    bool stop_= false;
    bool started_ = false;
    size_t thread_num_;

};
