#include "starq/thread_runner.hpp"

namespace starq
{

    bool ThreadRunner::start()
    {
        if (isRunning())
            return false;

        state_ = RUNNING;
        std::thread([this]()
                    {
                        run();
                        state_ = STOPPED; })
            .detach();
        return true;
    }

    bool ThreadRunner::stop()
    {
        if (!isRunning())
            return false;

        std::lock_guard<std::mutex> lock(mutex_);
        state_ = STOPPING;
        return true;
    }

    void ThreadRunner::wait()
    {
        while (!isStopped())
        {
            std::this_thread::sleep_for(std::chrono::microseconds(10));
        }
    }

    bool ThreadRunner::isRunning()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return state_ == RUNNING;
    }

    bool ThreadRunner::isStopping()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return state_ == STOPPING;
    }

    bool ThreadRunner::isStopped()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return state_ == STOPPED;
    }

}