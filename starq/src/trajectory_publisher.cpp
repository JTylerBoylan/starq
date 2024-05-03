#include "starq/trajectory_publisher.hpp"

#include <iostream>

namespace starq
{

    TrajectoryPublisher::TrajectoryPublisher(starq::LegCommandPublisher::Ptr leg_command_publisher)
        : leg_command_publisher_(leg_command_publisher),
          sleep_duration_us_(100)
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

            const auto current_time = leg_command_publisher_->getLocalization()->getCurrentTime();
            const auto release_time = leg_command->release_time;
            const auto delta_time = release_time - current_time;
            
            if (delta_time.count() > 0)
            {
                std::this_thread::sleep_for(delta_time);
            }

            leg_command_publisher_->sendCommand(leg_command);
        }

        stop();
    }

}