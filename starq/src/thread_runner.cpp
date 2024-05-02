#include "starq/thread_runner.hpp"

namespace starq
{

    bool ThreadRunner::start()
    {
        if (isRunning())
            return false;

        std::lock_guard<std::mutex> lock(mutex_);
        running_ = true;
        std::thread(&ThreadRunner::run, this).detach();
        return true;
    }

    bool ThreadRunner::stop()
    {
        if (!isRunning())
            return false;

        std::lock_guard<std::mutex> lock(mutex_);
        running_ = false;
        return true;
    }

    bool ThreadRunner::isRunning()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return running_;
    }

}