#ifndef STARQ_MPC__GAIT_HPP_
#define STARQ_MPC__GAIT_HPP_

#include <memory>
#include <vector>
#include <string>
#include <map>

#include "starq/mpc/mpc_types.hpp"

#define GAIT_RESOLUTION 1000

namespace starq::mpc
{
    class Gait
    {
    public:
        using Ptr = std::shared_ptr<Gait>;

        Gait();

        ~Gait();

        void load(const std::string &filename);

        void setFrequency(const double &frequency)
        {
            duration_ = milliseconds(static_cast<int>(1000.0 / frequency));
        }

        milliseconds getDuration() const
        {
            return duration_;
        }

        StanceState getStanceState(const milliseconds &time) const 
        {
            if (stance_pattern_.empty())
            {
                return StanceState();
            }

            auto time_scaled = milliseconds((GAIT_RESOLUTION * time.count()) / duration_.count());

            auto stance = stance_pattern_.lower_bound(time_scaled);

            if (stance == stance_pattern_.end())
            {
                return stance_pattern_.rbegin()->second;
            }

            return stance->second;
        }

    private:
        milliseconds duration_;
        std::map<milliseconds, StanceState> stance_pattern_;
    };
}

#endif