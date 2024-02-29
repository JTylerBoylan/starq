#include "starq/mpc/mpc_planner.hpp"

#include <iostream>

namespace starq::mpc
{

    MPCPlanner::MPCPlanner(starq::Robot::Ptr robot)
        : robot_(robot),
          gait_sequencer_(std::make_shared<GaitSequencer>(robot->getLocalization())),
          com_planner_(std::make_shared<CenterOfMassPlanner>(robot->getLocalization())),
          foothold_planner_(std::make_shared<FootholdPlanner>(robot->getLegs(), robot->getHipLocations(), robot->getLocalization()))
    {
        time_step_ = milliseconds(50);
        window_size_ = 21;
    }

    MPCPlanner::~MPCPlanner()
    {
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

        config.mass = robot_->getBodyMass();
        config.inertia = robot_->getBodyInertia();
        config.gravity = robot_->getGravity();
        config.height = robot_->getBodyHeight();

        config.time_step = time_step_;
        config.window_size = window_size_;

        config.stance_trajectory.resize(config.window_size);
        config.com_trajectory.resize(config.window_size);
        config.foothold_trajectory.resize(config.window_size);
        config.timing_trajectory.resize(config.window_size);

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