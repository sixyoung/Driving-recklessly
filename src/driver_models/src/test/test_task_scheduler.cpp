#include "driver_models/task_scheduler.h"
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    // 创建 TaskScheduler，2 个 worker 线程
    TaskScheduler scheduler(5);

    auto now = std::chrono::steady_clock::now();

    // 任务 A：延迟 300ms 开始，周期 300ms
    Task taskA;
    taskA.vehicle_id = 1;
    taskA.period = std::chrono::milliseconds(100);
    taskA.next_deadline = now + std::chrono::milliseconds(100);
    taskA.func = []() {
        std::cout << "[Task A] executed at " 
                  << std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::steady_clock::now().time_since_epoch()
                     ).count()
                  << " ms" << std::endl;
    };
    scheduler.addTask(taskA);

    // 任务 B：延迟 100ms 开始，周期 100ms
    Task taskB;
    taskB.vehicle_id = 2;
    taskB.period = std::chrono::milliseconds(100);
    taskB.next_deadline = now + std::chrono::milliseconds(100);
    taskB.func = []() {
        std::cout << "[Task B] executed at " 
                  << std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::steady_clock::now().time_since_epoch()
                     ).count()
                  << " ms" << std::endl;
    };
    scheduler.addTask(taskB);

    // 任务 C：延迟 200ms 开始，周期 200ms
    Task taskC;
    taskC.vehicle_id = 3;
    taskC.period = std::chrono::milliseconds(200);
    taskC.next_deadline = now + std::chrono::milliseconds(200);
    taskC.func = []() {
        std::cout << "[Task C] executed at " 
                  << std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::steady_clock::now().time_since_epoch()
                     ).count()
                  << " ms" << std::endl;
    };
    scheduler.addTask(taskC);

    // 主线程等 1 秒钟，让 worker 执行任务
    std::this_thread::sleep_for(std::chrono::seconds(1));

    return 0;
}
