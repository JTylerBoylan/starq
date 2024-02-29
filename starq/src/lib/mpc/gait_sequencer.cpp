#include "starq/mpc/gait_sequencer.hpp"

namespace starq::mpc
{
    GaitSequencer::GaitSequencer(starq::slam::Localization::Ptr localization)
        : localization_(localization),
          start_time_(localization->getCurrentTime()),
          current_gait_(nullptr),
          next_gait_(nullptr)
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
            start_time_ = localization_->getCurrentTime();
        }

        next_gait_ = gait;
    }

    bool GaitSequencer::sync()
    {
        if (next_gait_ == nullptr)
        {
            return false;
        }

        const auto current_time = localization_->getCurrentTime();
        if (current_time >= start_time_ + current_gait_->getDuration())
        {
            current_gait_ = next_gait_;
            start_time_ = current_time;
        }
        return true;
    }

    bool GaitSequencer::configure(MPCConfiguration &config) const
    {
        auto time = localization_->getCurrentTime();
        config.stance_trajectory[0] = getStanceState(time);
        
        for (size_t i = 1; i < config.window_size; i++)
        {
            time += config.time_step;
            config.stance_trajectory[i] = getStanceState(time);
            config.com_trajectory[i].linear_velocity = getGait(time)->getLinearVelocity();
            config.com_trajectory[i].angular_velocity = getGait(time)->getAngularVelocity();
            config.timing_trajectory[i].stance_duration = getGait(time)->getStanceDuration();
            config.timing_trajectory[i].swing_duration = getGait(time)->getSwingDuration();
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

    Gait::Ptr GaitSequencer::getGait(const milliseconds &time) const
    {
        if (current_gait_ == nullptr)
        {
            return nullptr;
        }

        const auto end_time = start_time_ + current_gait_->getDuration();
        if (time < end_time)
        {
            return current_gait_;
        }
        else
        {
            return next_gait_;
        }
    }

}