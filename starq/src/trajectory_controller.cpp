#include "starq/trajectory_controller.hpp"

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

namespace starq
{

    TrajectoryController::TrajectoryController(LegCommandPublisher::Ptr leg_command_publisher)
        : leg_command_publisher_(leg_command_publisher),
          frequency_(1.0),
          num_loops_(1)
    {
    }

    TrajectoryController::~TrajectoryController()
    {
        if (isRunning())
        {
            stop();
            wait();
        }
    }

    bool TrajectoryController::setFrequency(const Float frequency)
    {
        if (frequency <= 0.0f)
        {
            std::cerr << "Frequency must be greater than 0." << std::endl;
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        frequency_ = frequency;
        return true;
    }

    bool TrajectoryController::setNumLoops(const size_t num_loops)
    {
        if (num_loops == 0)
        {
            std::cerr << "Number of loops must be greater than 0." << std::endl;
            return false;
        }

        if (isRunning())
        {
            std::cerr << "Cannot set number of loops while running." << std::endl;
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        num_loops_ = num_loops;
        return true;
    }

    bool TrajectoryController::setTrajectory(const Trajectory &trajectory)
    {
        if (trajectory.empty())
        {
            std::cerr << "Trajectory is empty." << std::endl;
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        trajectory_ = trajectory;
        sortTrajectory();
        return true;
    }

    bool TrajectoryController::setTrajectory(const std::string &file_path)
    {
        if (trajectory_file_cache_.find(file_path) == trajectory_file_cache_.end())
        {
            Trajectory trajectory;
            if (!loadTrajectoryFromFile(file_path, trajectory))
            {
                std::cerr << "Failed to load trajectory from file: " << file_path << std::endl;
                return false;
            }

            trajectory_file_cache_[file_path] = trajectory;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        trajectory_ = trajectory_file_cache_[file_path];
        sortTrajectory();
        return true;
    }

    Float TrajectoryController::getFrequency()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return frequency_;
    }

    size_t TrajectoryController::getNumLoops()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return num_loops_;
    }

    Trajectory TrajectoryController::getTrajectory()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return trajectory_;
    }

    void TrajectoryController::run()
    {
        const size_t num_loops = getNumLoops();
        for (size_t i = 0; i < num_loops && isRunning(); i++)
        {
            const Float frequency = getFrequency();
            const auto trajectory = getTrajectory();
            const auto time_start = leg_command_publisher_->getLocalization()->getCurrentTime();

            for (auto &leg_command : trajectory)
            {
                const auto release_time = time_start + leg_command->delay / frequency;
                const auto current_time = leg_command_publisher_->getLocalization()->getCurrentTime();
                const auto delta_time = release_time - current_time;

                if (delta_time.count() > 0)
                {
                    std::this_thread::sleep_for(delta_time);
                }

                leg_command_publisher_->sendCommand(leg_command);
            }
        }
    }

    bool TrajectoryController::loadTrajectoryFromFile(const std::string &file_path, Trajectory &trajectory)
    {
        trajectory.clear();

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

            int leg_id;
            time_t delay_time;

            LegCommand::Ptr command = std::make_shared<LegCommand>();
            if (!(iss >>
                  delay_time >>
                  leg_id >>
                  command->control_mode >> command->input_mode >>
                  command->target_position.x() >> command->target_position.y() >> command->target_position.z() >>
                  command->target_velocity.x() >> command->target_velocity.y() >> command->target_velocity.z() >>
                  command->target_force.x() >> command->target_force.y() >> command->target_force.z()))
            {
                std::cerr << "Error reading line " << line << std::endl;
                return false;
            }

            command->delay = std::chrono::milliseconds(time_t(delay_time));
            command->leg_id = leg_id;

            trajectory.push_back(command);
        }

        return true;
    }

    void TrajectoryController::sortTrajectory()
    {
        std::sort(trajectory_.begin(), trajectory_.end(),
                  [](const LegCommand::Ptr &lhs, const LegCommand::Ptr &rhs)
                  {
                      return lhs->delay < rhs->delay;
                  });
    }

}