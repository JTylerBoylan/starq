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
        enum ThreadState {RUNNING, STOPPING, STOPPED};

        /// @brief Start the thread
        /// @return If the thread was successfully started
        virtual bool start();

        /// @brief Stop the thread
        /// @return If the thread was successfully stopped
        virtual bool stop();

        /// @brief Wait for the thread to finish
        virtual void wait();

        /// @brief Check if the thread is running
        /// @return If the thread is running
        bool isRunning();

        /// @brief Check if the thread is stopping
        /// @return If the thread is stopping
        bool isStopping();

        /// @brief Check if the thread is stopped
        /// @return If the thread is stopped
        bool isStopped();

    protected:
        virtual void run() = 0;
        std::mutex mutex_;

    private:
        ThreadState state_ = STOPPED;
    };

}

#endif