#include "starq/trajectory_publisher.hpp"

#include <iostream>

namespace starq
{

    TrajectoryPublisher::TrajectoryPublisher(starq::LegCommandPublisher::Ptr leg_command_publisher)
        : leg_command_publisher_(leg_command_publisher),
          sleep_duration_us_(1000)
    {
    }

    TrajectoryPublisher::~TrajectoryPublisher()
    {
    }

    bool TrajectoryPublisher::runTrajectory(const std::vector<starq::LegCommand::Ptr> &trajectory)
    {

        if (isRunning())
        {
            std::cerr << "Trajectory publisher is already running" << std::endl;
            return false;
        }

        for (auto &leg_command : trajectory)
        {
            leg_command->stamp(leg_command_publisher_->getLocalization()->getCurrentTime());
        }

        trajectory_ = trajectory;

        std::sort(trajectory_.begin(), trajectory_.end(),
                  [](const starq::LegCommand::Ptr &lhs, const starq::LegCommand::Ptr &rhs)
                  {
                      return lhs->release_time < rhs->release_time;
                  });

        start();

        return true;
    }

    void TrajectoryPublisher::run()
    {

        for (auto &leg_command : trajectory_)
        {

            while (leg_command->release_time > leg_command_publisher_->getLocalization()->getCurrentTime())
            {
                std::this_thread::sleep_for(std::chrono::microseconds(sleep_duration_us_));
            }

            leg_command_publisher_->sendCommand(leg_command);

            std::this_thread::sleep_for(std::chrono::microseconds(sleep_duration_us_));
        }

        stop();
    }

}