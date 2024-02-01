#include "starq/trajectory_publisher.hpp"

#include <iostream>

namespace starq
{

    TrajectoryPublisher::TrajectoryPublisher(starq::LegCommandPublisher::Ptr leg_command_publisher)
        : leg_command_publisher_(leg_command_publisher),
          frequency_(100)
    {
    }

    TrajectoryPublisher::~TrajectoryPublisher()
    {
    }

    bool TrajectoryPublisher::setTrajectory(const std::vector<starq::LegCommand::Ptr> &trajectory)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        trajectory_ = trajectory;
    }

    bool TrajectoryPublisher::setFrequency(int frequency)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        frequency_ = frequency;
    }

    void TrajectoryPublisher::run()
    {
        const int frequency = frequency_;
        const std::vector<starq::LegCommand::Ptr> trajectory = trajectory_;
        const int sleep_duration_us = 1E6 / frequency;

        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto &leg_cmd : trajectory)
        {
            leg_command_publisher_->sendCommand(leg_cmd);
            std::this_thread::sleep_for(std::chrono::microseconds(sleep_duration_us));
        }

        stop();
    }
}