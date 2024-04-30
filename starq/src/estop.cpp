#include "starq/estop.hpp"

namespace starq
{

    bool Estop::estop_init_ = false;
    std::vector<std::function<void(int)>> Estop::estop_functions_;
    std::mutex Estop::mutex_;

    Estop::Estop()
    {
        init_estop_handler();
        std::lock_guard<std::mutex> lock(mutex_);
        estop_functions_.push_back([this](int sig) { this->estop(sig); });
    }

    void Estop::init_estop_handler()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (estop_init_)
            return;

        struct sigaction sa;
        sa.sa_handler = estop_handler;
        sigemptyset(&sa.sa_mask);
        sa.sa_flags = 0;

        sigaction(SIGINT, &sa, nullptr);
        sigaction(SIGTERM, &sa, nullptr);

        estop_init_ = true;
    }

    void Estop::estop_handler(int sig)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto &estop_function : estop_functions_)
        {
            estop_function(sig);
        }
        exit(EXIT_FAILURE);
    }

}