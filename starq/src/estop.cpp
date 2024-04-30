#include "starq/estop.hpp"

namespace starq
{

    bool Estop::estop_init_ = false;
    std::vector<std::function<void(int)>> Estop::estop_functions_;

    Estop::Estop()
    {
        init_estop_handler();
        estop_functions_.push_back(std::bind(&Estop::estop, this, std::placeholders::_1));
    }

    void Estop::init_estop_handler()
    {
        if (estop_init_)
            return;
        signal(SIGINT, estop_handler);
        signal(SIGTERM, estop_handler);
        estop_init_ = true;
    }

    void Estop::estop_handler(int sig)
    {
        for (const auto &estop_function : estop_functions_)
        {
            estop_function(sig);
        }
        std::raise(SIGKILL);
    }

}