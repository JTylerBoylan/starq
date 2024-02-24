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

        trajectory_queue_ = std::priority_queue<starq::LegCommand::Ptr,
                                                std::vector<starq::LegCommand::Ptr>,
                                                TrajectoryComparator>(trajectory.begin(), trajectory.end());

        for (auto &leg_command : trajectory)
        {
            leg_command->stamp();
        }

        start();

        return true;
    }

    bool TrajectoryPublisher::isEmpty()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return trajectory_queue_.empty();
    }

    void TrajectoryPublisher::run()
    {

        while (!isEmpty())
        {

            auto next = getNext();

            time_point_t release_time = next->release_time;
            time_point_t now = std::chrono::high_resolution_clock::now();

            auto duration = release_time - now;
            if (duration.count() > 0)
            {
                std::this_thread::sleep_for(duration);
            }

            leg_command_publisher_->sendCommand(next);

            std::this_thread::sleep_for(std::chrono::microseconds(sleep_duration_us_));
        }

        stop();
    }

    LegCommand::Ptr TrajectoryPublisher::getNext()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto top = trajectory_queue_.top();
        trajectory_queue_.pop();
        return top;
    }
}