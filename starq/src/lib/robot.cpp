#include "starq/robot.hpp"

#include <iostream>

namespace starq
{
    void Robot::setup()
    {
        setupMotors();
        setupLegs();
        setupLocalization();
        setupRobotDynamics();
        setupMPCSolver();
        setupLegCommandPublisher();
        setupTrajectoryFileReader();
        setupTrajectoryPublisher();
        setupMPCConfiguration();
        setupMPCController();
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

    void Robot::setupMPCConfiguration()
    {
        mpc_configuration_ = std::make_shared<mpc::MPCConfiguration>(legs_, robot_parameters_, localization_);
        mpc_configuration_->setTimeStep(milliseconds(50));
        mpc_configuration_->setWindowSize(21);
    }

    void Robot::setupMPCController()
    {
        mpc_controller_ = std::make_shared<mpc::MPCController>(mpc_configuration_, mpc_solver_, publisher_);
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

    bool Robot::startMPC()
    {
        return mpc_controller_->start();
    }

    bool Robot::stopMPC()
    {
        return mpc_controller_->stop();
    }

    void Robot::setNextGait(mpc::Gait::Ptr gait)
    {
        mpc_configuration_->setNextGait(gait);
    }

}