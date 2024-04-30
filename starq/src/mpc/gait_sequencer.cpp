#include "starq/mpc/gait_sequencer.hpp"

#include <iostream>

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

    bool GaitSequencer::configure(const size_t N, const milliseconds dt,
                                  StanceTrajectory &stance_traj,
                                  GaitSequence &gait_seq) const
    {
        stance_traj.resize(N);
        gait_seq.resize(N);

        const auto start_time = localization_->getCurrentTime();
        for (size_t i = 0; i < N; i++)
        {
            auto time = start_time + i * dt;
            stance_traj[i] = getStanceState(time);
            gait_seq[i] = getGait(time);
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
        if (time <= start_time_)
        {
            return current_gait_->getStanceState(milliseconds(1));
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