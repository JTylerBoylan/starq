#ifndef STARQ__THREAD_RUNNER_HPP
#define STARQ__THREAD_RUNNER_HPP

#include <thread>
#include <mutex>
#include <functional>
#include <vector>
#include <csignal>

namespace starq
{

    /// @brief Abstract class for running a thread
    class ThreadRunner
    {
    public:
        /// @brief Start the thread
        /// @return If the thread was successfully started
        virtual bool start();

        /// @brief Stop the thread
        /// @return If the thread was successfully stopped
        virtual bool stop();

        /// @brief Check if the thread is running
        /// @return If the thread is running
        bool isRunning();

    protected:
        virtual void run() = 0;
        std::mutex mutex_;

    private:
        bool running_ = false;
    };

}

#endif