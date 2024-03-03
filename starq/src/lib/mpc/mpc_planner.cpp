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
        time_step_ = milliseconds(0);
        window_size_ = 0;
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

    void MPCPlanner::setStateWeights(const Vector3f &position_weights,
                                     const Vector3f &orientation_weights,
                                     const Vector3f &linear_velocity_weights,
                                     const Vector3f &angular_velocity_weights)
    {
        position_weights_ = position_weights;
        orientation_weights_ = orientation_weights;
        linear_velocity_weights_ = linear_velocity_weights;
        angular_velocity_weights_ = angular_velocity_weights;
    }

    void MPCPlanner::setControlWeights(const Vector3f &force_weights)
    {
        force_weights_ = force_weights;
    }

    void MPCPlanner::setControlBounds(const float &fz_min, const float &fz_max)
    {
        fz_min_ = fz_min;
        fz_max_ = fz_max;
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

        if (time_step_ == milliseconds(0))
        {
            std::cerr << "Time step not set" << std::endl;
            return false;
        }

        config.mass = robot_->getBodyMass();
        config.inertia = robot_->getBodyInertia();
        config.gravity = robot_->getGravity();
        config.height = robot_->getBodyHeight();
        config.friction_coeff = robot_->getFootFriction();

        config.time_step = time_step_;
        config.window_size = window_size_;

        config.position_weights = position_weights_;
        config.orientation_weights = orientation_weights_;
        config.linear_velocity_weights = linear_velocity_weights_;
        config.angular_velocity_weights = angular_velocity_weights_;
        config.force_weights = force_weights_;

        config.fz_min = fz_min_;
        config.fz_max = fz_max_;

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