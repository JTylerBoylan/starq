#ifndef STARQ__TRAJECTORY_PUBLISHER_HPP
#define STARQ__TRAJECTORY_PUBLISHER_HPP

#include <queue>

#include "starq/leg_command_publisher.hpp"
#include "starq/thread_runner.hpp"

namespace starq
{

    /// @brief Publishes a leg trajectory at a fixed frequency
    class TrajectoryPublisher : public ThreadRunner
    {
    public:
        using Ptr = std::shared_ptr<TrajectoryPublisher>;

        /// @brief Create a trajectory publisher.
        /// @param leg_command_publisher Leg command publisher.
        TrajectoryPublisher(starq::LegCommandPublisher::Ptr leg_command_publisher);

        /// @brief Destroy the trajectory publisher.
        ~TrajectoryPublisher();

        /// @brief Run the trajectory.
        /// @param trajectory Vector of leg commands.
        /// @return True if the trajectory was ran successfully, false otherwise.
        bool runTrajectory(const std::vector<starq::LegCommand::Ptr> &trajectory);

        /// @brief Set the sleep duration between commands.
        /// @param sleep_duration_us Sleep duration in microseconds.
        void setSleepDuration(const time_t sleep_duration_us) { sleep_duration_us_ = sleep_duration_us; }

        /// @brief Check if the trajectory is empty.
        /// @return True if the trajectory is empty, false otherwise.
        bool isEmpty();

    private:
        /// @brief Run the trajectory publisher.
        void run() override;

        /// @brief Get the next leg command from the trajectory.
        /// @return Leg command.
        LegCommand::Ptr getNext();

        /// @brief Comparator for leg commands.
        class TrajectoryComparator
        {
        public:
            bool operator()(const starq::LegCommand::Ptr &lhs, const starq::LegCommand::Ptr &rhs)
            {
                return lhs->release_time > rhs->release_time;
            }
        };

        std::priority_queue<starq::LegCommand::Ptr,
                            std::vector<starq::LegCommand::Ptr>,
                            TrajectoryComparator>
            trajectory_queue_;

        starq::LegCommandPublisher::Ptr leg_command_publisher_;
        time_t sleep_duration_us_;
    };

}

#endif