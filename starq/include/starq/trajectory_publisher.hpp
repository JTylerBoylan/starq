#ifndef STARQ__TRAJECTORY_PUBLISHER_HPP
#define STARQ__TRAJECTORY_PUBLISHER_HPP

#include "starq/leg_command_publisher.hpp"
#include "starq/thread_runner.hpp"

namespace starq 
{

    class TrajectoryPublisher : public ThreadRunner
    {
        public:
            using Ptr = std::shared_ptr<TrajectoryPublisher>;

            TrajectoryPublisher(starq::LegCommandPublisher::Ptr leg_command_publisher);

            ~TrajectoryPublisher();

            bool setTrajectory(const std::vector<starq::LegCommand::Ptr> &trajectory);

            bool setFrequency(int frequency);

        private:
            void run() override;

            starq::LegCommandPublisher::Ptr leg_command_publisher_;

            int frequency_;
            std::vector<starq::LegCommand::Ptr> trajectory_;
    };

}

#endif