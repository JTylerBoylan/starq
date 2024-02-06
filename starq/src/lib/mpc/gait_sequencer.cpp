#include "starq/mpc/gait_sequencer.hpp"

namespace starq::mpc
{
    GaitSequencer::GaitSequencer()
    {
    }

    GaitSequencer::~GaitSequencer()
    {
    }

    void GaitSequencer::setNextGait(Gait::Ptr gait)
    {
        if (current_gait_ == nullptr)
        {
            current_gait_ = gait;
            start_time_ = getCurrentTime();
        }

        next_gait_ = gait;
    }

    bool GaitSequencer::sync()
    {
        if (next_gait_ == nullptr)
        {
            return false;
        }

        const auto current_time = getCurrentTime();
        if (current_time >= start_time_ + current_gait_->getDuration())
        {
            current_gait_ = next_gait_;
            start_time_ = current_time;
        }
        return true;
    }

    bool GaitSequencer::configure(MPCConfiguration &config) const
    {
        auto time = getCurrentTime();
        config.stance_trajectory[0] = getStanceState(time);
        
        for (int i = 1; i < config.window_size; i++)
        {
            time += config.time_step * i;
            config.stance_trajectory[i] = getStanceState(time);
            config.com_trajectory[i].linear_velocity = getLinearVelocity(time);
            config.com_trajectory[i].angular_velocity = getAngularVelocity(time);
        }

        return true;
    }

    StanceState GaitSequencer::getStanceState(const milliseconds &time) const
    {
        if (current_gait_ == nullptr)
        {
            return StanceState();
        }

        const auto end_time = start_time_ + current_gait_->getDuration();
        if (time < start_time_)
        {
            return current_gait_->getStanceState(milliseconds(0));
        }
        else if (time < end_time)
        {
            return current_gait_->getStanceState(time - start_time_);
        }
        else
        {
            return next_gait_->getStanceState((time - end_time) % next_gait_->getDuration());
        }
    }

    Vector3f GaitSequencer::getLinearVelocity(const milliseconds &time) const
    {
        if (current_gait_ == nullptr)
        {
            return Vector3f::Zero();
        }

        const auto end_time = start_time_ + current_gait_->getDuration();
        if (time < end_time)
        {
            return current_gait_->getLinearVelocity();
        }
        else
        {
            return next_gait_->getLinearVelocity();
        }
    }

    Vector3f GaitSequencer::getAngularVelocity(const milliseconds &time) const
    {
        if (current_gait_ == nullptr)
        {
            return Vector3f::Zero();
        }

        const auto end_time = start_time_ + current_gait_->getDuration();
        if (time < end_time)
        {
            return current_gait_->getAngularVelocity();
        }
        else
        {
            return next_gait_->getAngularVelocity();
        }
    }

}