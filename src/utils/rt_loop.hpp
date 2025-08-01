#ifndef RT_LOOP_HPP
#define RT_LOOP_HPP

#include <iostream>
#include <string>
#include <thread>
#include <functional>
#include <vector>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <pthread.h>
#include <sched.h>

namespace real_time_loop {

static constexpr int thread_priority = 99;
using period_t = std::chrono::duration<double>;

class loop {
public:
    loop(std::string name, period_t period, int bind_cpu = -1)
      : _name(std::move(name))
      , _period(period)
      , _bind_cpu(bind_cpu)
      , _is_running(false)
    {}

    virtual ~loop() noexcept {
        shutdown();
    }

    void start() {
        bool expected = false;
        if (!_is_running.compare_exchange_strong(expected, true)) {
            std::cout << "WARN: Loop '" << _name << "' is already running.\n";
            return;
        }
        _thread = std::thread(&loop::entry_func, this);
    }

    void shutdown() {
        bool expected = true;
        if (!_is_running.compare_exchange_strong(expected, false)) {
            std::cout << "WARN: Loop '" << _name << "' is not running.\n";
            return;
        }
        // wake up the thread if it's sleeping
        {
            std::lock_guard<std::mutex> lk(_mutex);
        }
        _cv.notify_all();

        if (_thread.joinable()) {
            _thread.join();
        }
    }

    // Checks if the loop's thread is currently running.
    // True if the loop is running, false otherwise.
    bool is_running() const {
        return _is_running.load(std::memory_order_relaxed);
    }

    virtual void function_cb() = 0;

private:
    void entry_func() {
        // name the thread for easier debugging
        pthread_setname_np(_thread.native_handle(), _name.c_str());

        // set CPU affinity
        if (_bind_cpu >= 0) {
            cpu_set_t cpuset;
            CPU_ZERO(&cpuset);
            CPU_SET(_bind_cpu, &cpuset);
            if (pthread_setaffinity_np(
                  _thread.native_handle(),
                  sizeof(cpuset),
                  &cpuset) != 0)
            {
                std::cerr << "ERROR: Failed to bind thread '"
                          << _name << "' to CPU " << _bind_cpu << "\n";
            }
        }

        // real-time priority
        sched_param param{};
        param.sched_priority = thread_priority;
        if (pthread_setschedparam(
              _thread.native_handle(),
              SCHED_FIFO,
              &param) != 0)
        {
            std::cerr << "ERROR: Failed to set RT priority for '"
                      << _name << "'\n";
        }

        auto next_time = std::chrono::steady_clock::now() + _period;
        while (_is_running.load(std::memory_order_acquire)) {
            function_cb();

            next_time += _period;
            std::unique_lock<std::mutex> lk(_mutex);
            // wake either at next_time or on shutdown()
            _cv.wait_until(lk, next_time, [this]() {
                return !_is_running.load(std::memory_order_acquire);
            });

            // deadline check
            auto now = std::chrono::steady_clock::now();
            if (now > next_time) {
                std::cout << "WARN: Loop '" << _name
                          << "' missed its deadline by "
                          << std::chrono::duration<double>(now - next_time).count()
                          << " s\n";
            }
        }
    }

    std::string               _name;
    period_t                  _period;
    int                       _bind_cpu;
    std::atomic<bool>         _is_running;
    std::thread               _thread;

    // for interruptible sleep
    std::mutex                _mutex;
    std::condition_variable   _cv;
};

class loop_func : public loop {
public:
    loop_func(const std::string& name,
              period_t           period,
              std::function<void()> cb)
      : loop(name, period)
      , _fp(std::move(cb))
    {}

    loop_func(const std::string& name,
              period_t           period,
              int                bind_cpu,
              std::function<void()> cb)
      : loop(name, period, bind_cpu)
      , _fp(std::move(cb))
    {}

    void function_cb() override {
        if (_fp) _fp();
    }

private:
    std::function<void()> _fp;
};

} // namespace real_time_loop

#endif // RT_LOOP_HPP
