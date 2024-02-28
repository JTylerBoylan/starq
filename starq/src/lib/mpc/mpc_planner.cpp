#include "starq/mpc/mpc_planner.hpp"

#include <iostream>

namespace starq::mpc
{

    MPCPlanner::MPCPlanner(std::vector<LegController::Ptr> legs,
                           starq::slam::Localization::Ptr localization)
        : gait_sequencer_(std::make_shared<GaitSequencer>(localization)),
          com_planner_(std::make_shared<CenterOfMassPlanner>(localization)),
          foothold_planner_(std::make_shared<FootholdPlanner>(legs, localization))
    {
        mass_ = 1.0;
        inertia_ = Eigen::Matrix3f::Identity();
        gravity_ = Eigen::Vector3f(0, 0, -9.81);
        time_step_ = milliseconds(50);
        window_size_ = 21;
    }

    MPCPlanner::~MPCPlanner()
    {
    }

    void MPCPlanner::setMass(const float &mass)
    {
        mass_ = mass;
    }

    void MPCPlanner::setInertia(const Matrix3f &inertia)
    {
        inertia_ = inertia;
    }

    void MPCPlanner::setGravity(const Vector3f &gravity)
    {
        gravity_ = gravity;
    }

    void MPCPlanner::setTimeStep(const milliseconds &time_step)
    {
        time_step_ = time_step;
    }

    void MPCPlanner::setWindowSize(const size_t &window_size)
    {
        window_size_ = window_size;
    }

    void MPCPlanner::setNextGait(Gait::Ptr gait)
    {
        gait_sequencer_->setNextGait(gait);
    }

    bool MPCPlanner::getConfiguration(MPCConfiguration &config)
    {
        if (window_size_ == 0)
        {
            std::cerr << "Window size not set" << std::endl;
            return false;
        }

        config.mass = mass_;
        config.inertia = inertia_;
        config.gravity = gravity_;

        config.time_step = time_step_;
        config.window_size = window_size_;

        config.stance_trajectory.resize(config.window_size);
        config.com_trajectory.resize(config.window_size);
        config.foothold_trajectory.resize(config.window_size);

        if (!gait_sequencer_->sync())
        {
            std::cerr << "Could not synchronize gait sequencer" << std::endl;
            return false;
        }

        if (!gait_sequencer_->configure(config))
        {
            std::cerr << "Gait sequencer could not run configuration" << std::endl;
            return false;
        }

        if (!com_planner_->configure(config))
        {
            std::cerr << "Center of mass planner could not run configuration" << std::endl;
            return false;
        }

        if (!foothold_planner_->configure(config))
        {
            std::cerr << "Foothold planner could not run configuration" << std::endl;
            return false;
        }

        return true;
    }

}