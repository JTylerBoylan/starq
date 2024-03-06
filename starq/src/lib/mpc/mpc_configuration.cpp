#include "starq/mpc/mpc_configuration.hpp"

#include <iostream>

namespace starq::mpc
{

    MPCConfiguration::MPCConfiguration(const std::vector<LegController::Ptr> &leg_controllers,
                                       const RobotDynamics::Ptr &robot_dynamics,
                                       const slam::Localization::Ptr &localization)
        : leg_controllers_(leg_controllers),
          robot_dynamics_(robot_dynamics),
          localization_(localization),
          is_ready_(false),
          gait_sequencer_(std::make_shared<GaitSequencer>(localization)),
          com_planner_(std::make_shared<CenterOfMassPlanner>(localization, robot_dynamics)),
          foothold_planner_(std::make_shared<FootholdPlanner>(leg_controllers, robot_dynamics, localization))
    {
        window_size_ = 11;
        time_step_ = milliseconds(100);
    }

    MPCConfiguration::~MPCConfiguration()
    {
    }

    std::vector<LegController::Ptr> MPCConfiguration::getLegControllers() const
    {
        return leg_controllers_;
    }

    RobotDynamics::Ptr MPCConfiguration::getRobotDynamics() const
    {
        return robot_dynamics_;
    }

    slam::Localization::Ptr MPCConfiguration::getLocalization() const
    {
        return localization_;
    }

    void MPCConfiguration::setTimeStep(const milliseconds &time_step)
    {
        time_step_ = time_step;
    }

    void MPCConfiguration::setWindowSize(const size_t &window_size)
    {
        window_size_ = window_size;
    }

    void MPCConfiguration::setNextGait(const Gait::Ptr &gait)
    {
        gait_sequencer_->setNextGait(gait);
        is_ready_ = true;
    }

    bool MPCConfiguration::isReady() const
    {
        return is_ready_;
    }

    bool MPCConfiguration::update()
    {
        if (!gait_sequencer_->sync())
        {
            std::cerr << "Could not synchronize gait sequencer" << std::endl;
            return false;
        }

        if (!gait_sequencer_->configure(window_size_, time_step_, stance_trajectory_, gait_sequence_))
        {
            std::cerr << "Could not configure gait sequencer" << std::endl;
            return false;
        }

        if (!com_planner_->configure(window_size_, time_step_, gait_sequence_, reference_trajectory_))
        {
            std::cerr << "Could not configure center of mass planner" << std::endl;
            return false;
        }

        if (!foothold_planner_->configure(window_size_, stance_trajectory_,
                                          gait_sequence_, reference_trajectory_, foothold_trajectory_))
        {
            std::cerr << "Could not configure foothold planner" << std::endl;
            return false;
        }

        n_legs_.resize(window_size_);
        for (size_t k = 0; k < window_size_; k++)
        {
            int c = 0;
            for (bool s : stance_trajectory_[k])
                if (s)
                    c++;
            n_legs_[k] = c;
        }

        return true;
    }

    float MPCConfiguration::getTimeStep() const
    {
        return time_step_.count() / 1000.0;
    }

    size_t MPCConfiguration::getWindowSize() const
    {
        return window_size_;
    }

    StanceState MPCConfiguration::getStanceState(const int node) const
    {
        return stance_trajectory_[node];
    }

    ReferenceState MPCConfiguration::getReferenceState(const int node) const
    {
        return reference_trajectory_[node];
    }

    FootholdState MPCConfiguration::getFootholdState(const int node) const
    {
        return foothold_trajectory_[node];
    }

    Gait::Ptr MPCConfiguration::getGait(const int node) const
    {
        return gait_sequence_[node];
    }

    size_t MPCConfiguration::getNumberOfLegs(const int node) const
    {
        return n_legs_[node];
    }

    ReferenceWeights MPCConfiguration::getReferenceWeights(const int node) const
    {
        return gait_sequence_[node]->getReferenceWeights();
    }

    ForceWeights MPCConfiguration::getForceWeights(const int node) const
    {
        return gait_sequence_[node]->getForceWeights();
    }

    Vector3f MPCConfiguration::getGravity() const
    {
        return robot_dynamics_->getGravity();
    }

    float MPCConfiguration::getFrictionCoefficient() const
    {
        return robot_dynamics_->getFootFriction();
    }

    float MPCConfiguration::getForceZMin() const
    {
        return robot_dynamics_->getForceZMin();
    }

    float MPCConfiguration::getForceZMax() const
    {
        return robot_dynamics_->getForceZMax();
    }

    float MPCConfiguration::getMass() const
    {
        return robot_dynamics_->getBodyMass();
    }

    Matrix3f MPCConfiguration::getInertia() const
    {
        return robot_dynamics_->getBodyInertia();
    }

}