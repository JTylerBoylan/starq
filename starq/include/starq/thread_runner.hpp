#ifndef STARQ__THREAD_RUNNER_HPP
#define STARQ__THREAD_RUNNER_HPP

#include <thread>
#include <mutex>

namespace starq
{
    class ThreadRunner
    {
    public:
        virtual bool start()
        {
            if (isRunning())
                return false;

            running_ = true;
            std::thread(&ThreadRunner::run, this).detach();
            return true;
        }

        virtual bool stop()
        {
            if (!isRunning())
                return false;

            running_ = false;
            return true;
        }

        bool isRunning()
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return running_;
        }

    protected:
        virtual void run() = 0;
        std::mutex mutex_;

    private:
        bool running_ = false;
    };
}

#endif