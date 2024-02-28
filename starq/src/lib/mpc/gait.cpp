#include "starq/mpc/gait.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace starq::mpc
{
    Gait::Gait()
        : duration_(1000),
          linear_velocity_(Vector3f(1, 0, 0)),
          angular_velocity_(Vector3f::Zero())
    {
    }

    Gait::~Gait()
    {
    }

    bool Gait::load(const std::string &file_path)
    {
        stance_pattern_.clear();

        std::ifstream file(file_path);

        if (!file.is_open())
        {
            std::cerr << "Could not open file " << file_path << std::endl;
            return false;
        }

        std::string line;
        while (std::getline(file, line))
        {
            std::istringstream iss(line);

            time_t time_ms;
            StanceState stance;

            if (!(iss >> time_ms))
            {
                std::cerr << "Error reading line " << line << std::endl;
                return false;
            }

            bool stance_i;
            while (iss >> stance_i)
            {
                stance.push_back(stance_i);
            }

            stance_pattern_[milliseconds(time_ms)] = stance;
        }

        return true;
    }

    void Gait::setFrequency(const double &frequency)
    {
        duration_ = milliseconds(static_cast<int>(1000.0 / frequency));
    }

    void Gait::setVelocity(const Vector3f &linear_velocity, const Vector3f &angular_velocity)
    {
        linear_velocity_ = linear_velocity;
        angular_velocity_ = angular_velocity;
    }

    milliseconds Gait::getDuration() const
    {
        return duration_;
    }

    Vector3f Gait::getLinearVelocity() const
    {
        return linear_velocity_;
    }

    Vector3f Gait::getAngularVelocity() const
    {
        return angular_velocity_;
    }

    StanceState Gait::getStanceState(const milliseconds &time) const
    {
        if (stance_pattern_.empty())
        {
            return StanceState();
        }

        auto time_scaled = milliseconds((GAIT_RESOLUTION * time.count()) / duration_.count());

        auto stance = stance_pattern_.lower_bound(time_scaled);

        if (stance == stance_pattern_.end())
        {
            return StanceState();
        }

        return stance->second;
    }
}