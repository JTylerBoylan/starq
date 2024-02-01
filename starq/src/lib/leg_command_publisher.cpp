#include "starq/leg_command_publisher.hpp"

#include <iostream>

namespace starq
{

    LegCommandPublisher::LegCommandPublisher(const std::vector<LegController::Ptr> leg_controllers)
        : leg_controllers_(leg_controllers),
          stop_on_fail_(true),
          sleep_duration_us_(5000)
    {
        start();
    }

    LegCommandPublisher::~LegCommandPublisher()
    {
        if (isRunning())
            stop();
    }

    void LegCommandPublisher::sendCommand(LegCommand::Ptr leg_command)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (leg_command->leg_id >= leg_controllers_.size())
        {
            std::cerr << "Leg ID out of range." << std::endl;
            return;
        }

        leg_command_map_[leg_command->leg_id] = leg_command;
    }

    void LegCommandPublisher::clear()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        leg_command_map_.clear();
    }

    void LegCommandPublisher::run()
    {
        using namespace std::chrono;

        while (isRunning())
        {

            bool command_success = true;
            for (auto iter = leg_command_map_.begin(); iter != leg_command_map_.end() && command_success; ++iter)
            {

                LegController::Ptr leg_controller = leg_controllers_[iter->first];

                LegCommand leg_cmd;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    leg_cmd = *(iter->second);
                }

                if (!leg_controller->setControlMode(leg_cmd.control_mode, leg_cmd.input_mode))
                {
                    command_success = false;
                    break;
                }

                switch (leg_cmd.control_mode)
                {
                case ControlMode::POSITION:
                    command_success = leg_controller->setFootPosition(leg_cmd.target_position,
                                                                      leg_cmd.target_velocity,
                                                                      leg_cmd.target_force);
                    break;
                case ControlMode::VELOCITY:
                    command_success = leg_controller->setFootVelocity(leg_cmd.target_velocity,
                                                                      leg_cmd.target_force);
                    break;
                case ControlMode::TORQUE:
                    command_success = leg_controller->setFootForce(leg_cmd.target_force);
                    break;
                }
            }

            if (!command_success)
            {
                std::cerr << "Failed to send leg command." << std::endl;

                if (stop_on_fail_)
                {
                    if (!stop())
                    {
                        std::cerr << "Failed to stop leg command publisher." << std::endl;
                    }
                }
            }

            std::this_thread::sleep_for(microseconds(sleep_duration_us_));
        }
    }
}
