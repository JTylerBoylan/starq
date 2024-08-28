#include "starq/logger.hpp"

namespace starq
{
    Logger::Logger(const std::string &filename, const std::string &dir)
    {
        std::string file_path = dir + "/" + filename;
        file_.open(file_path, std::ios::out);
    }

    Logger::~Logger()
    {
        file_.close();
    }

    void Logger::log(const std::string &data)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        file_ << data;
    }
}