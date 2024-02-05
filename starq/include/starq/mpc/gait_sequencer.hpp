#ifndef STARQ_MPC__GAIT_SEQUENCER_HPP_
#define STARQ_MPC__GAIT_SEQUENCER_HPP_

#include <memory>

#include "starq/mpc/gait.hpp"

namespace starq::mpc
{
    class GaitSequencer
    {
    public:
        using Ptr = std::shared_ptr<GaitSequencer>;

        GaitSequencer();

        ~GaitSequencer();

        void setNextGait(Gait::Ptr gait)
        {
            if (current_gait_ == nullptr)
            {
                current_gait_ = gait;
                start_time_ = getCurrentTime();
            }

            next_gait_ = gait;
        }

        bool sync()
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

        StanceState getStanceState(const milliseconds &time) const
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

        StanceTrajectory getStanceTrajectory(const float time_step, const size_t window_size) const
        {
            StanceTrajectory stances;
            stances.reserve(window_size);

            const auto current_time = getCurrentTime();
            for (int i = 0; i < window_size; i++)
            {
                stances.push_back(getStanceState(current_time + milliseconds(static_cast<int>(time_step * i))));
            }

            return stances;
        }

    private:

        milliseconds start_time_;

        Gait::Ptr current_gait_;
        Gait::Ptr next_gait_;
    };
}

#endif