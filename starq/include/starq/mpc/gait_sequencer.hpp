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
                end_time_ = start_time_ + current_gait_->getDuration();
            }

            next_gait_ = gait;
        }

        void sync()
        {
            const auto current_time = getCurrentTime();

            if (next_gait_ == nullptr)
            {
                return;
            }

            if (current_time >= end_time_)
            {
                current_gait_ = next_gait_;
                start_time_ = current_time;
                end_time_ = current_time + current_gait_->getDuration();
            }
        }

        StanceState getStanceState(const milliseconds &time) const
        {
            if (current_gait_ == nullptr)
            {
                return StanceState();
            }

            if (time < start_time_)
            {
                return current_gait_->getStanceState(milliseconds(0));
            }
            else if (time < end_time_)
            {
                return current_gait_->getStanceState(time - start_time_);
            }
            else
            {
                return next_gait_->getStanceState((time - end_time_) % next_gait_->getDuration());
            }
        }

    private:
        inline milliseconds getCurrentTime() const
        {
            return duration_cast<milliseconds>(system_clock::now().time_since_epoch());
        }

        milliseconds start_time_;
        milliseconds end_time_;

        Gait::Ptr current_gait_;
        Gait::Ptr next_gait_;
    };
}

#endif