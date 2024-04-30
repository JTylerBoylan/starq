#ifndef STARQ_ESTOP_HPP_
#define STARQ_ESTOP_HPP_

#include <memory>
#include <functional>
#include <vector>
#include <csignal>

namespace starq
{
    /// @brief Abstract class for emergency stop callbacks
    class Estop
    {
    protected:

        /// @brief Constructor
        Estop();

        /// @brief E-stop callback
        /// @param sig Signal number
        virtual void estop(int sig) = 0;

    private:
        static void init_estop_handler();
        static void estop_handler(int sig);

        static bool estop_init_;
        static std::vector<std::function<void(int)>> estop_functions_;
    };
}

#endif