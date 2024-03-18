#include "starq/robot.hpp"

#include <iostream>

namespace starq
{
    void Robot::setup()
    {
        setupMotors();
        setupLegs();
        setupLocalization();
        setupLegCommandPublisher();
        setupTrajectoryFileReader();
        setupTrajectoryPublisher();
    }

    void Robot::setupLegCommandPublisher()
    {
        publisher_ = std::make_shared<LegCommandPublisher>(legs_);
    }

    void Robot::setupTrajectoryFileReader()
    {
        trajectory_file_reader_ = std::make_shared<TrajectoryFileReader>();
    }

    void Robot::setupTrajectoryPublisher()
    {
        trajectory_publisher_ = std::make_shared<TrajectoryPublisher>(publisher_);
    }

    bool Robot::setFootPosition(const uint8_t &leg_id, const Vector3 &position)
    {
        if (leg_id >= legs_.size())
        {
            std::cerr << "Invalid leg id: " << leg_id << std::endl;
            return false;
        }

        auto command = std::make_shared<LegCommand>();
        command->control_mode = ControlMode::POSITION;
        command->target_position = position;
        command->leg_id = leg_id;

        publisher_->sendCommand(command);
        return true;
    }

    bool Robot::setFootVelocity(const uint8_t &leg_id, const Vector3 &velocity)
    {
        if (leg_id >= legs_.size())
        {
            std::cerr << "Invalid leg id: " << leg_id << std::endl;
            return false;
        }

        auto command = std::make_shared<LegCommand>();
        command->control_mode = ControlMode::VELOCITY;
        command->target_velocity = velocity;
        command->leg_id = leg_id;

        publisher_->sendCommand(command);
        return true;
    }

    bool Robot::setFootForce(const uint8_t &leg_id, const Vector3 &force)
    {
        if (leg_id >= legs_.size())
        {
            std::cerr << "Invalid leg id: " << leg_id << std::endl;
            return false;
        }

        auto command = std::make_shared<LegCommand>();
        command->control_mode = ControlMode::TORQUE;
        command->target_force = force;
        command->leg_id = leg_id;

        publisher_->sendCommand(command);
        return true;
    }

    bool Robot::loadTrajectory(const std::string &file)
    {
        return trajectory_file_reader_->load(file);
    }

    bool Robot::startTrajectory()
    {
        return trajectory_publisher_->runTrajectory(trajectory_file_reader_->getTrajectory());
    }

    bool Robot::runTrajectory(const std::vector<LegCommand::Ptr> &trajectory)
    {
        return trajectory_publisher_->runTrajectory(trajectory);
    }

}