#ifndef STARQ__LOGGER_HPP_
#define STARQ__LOGGER_HPP_

#include <memory>
#include <fstream>

namespace starq
{
    // @brief Class for logging data
    class Logger
    {
    public:
        using Ptr = std::shared_ptr<Logger>;

        // @brief Constructor
        Logger(const std::string &filename, const std::string &dir = "/tmp");

        // @brief Destructor
        ~Logger();

        // @brief Log data
        void log(const std::string &data);

    private:
        std::ofstream file_;
    };
}

#endif