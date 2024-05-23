#ifndef STARQ__TRAJECTORY_CONTROLLER_HPP_
#define STARQ__TRAJECTORY_CONTROLLER_HPP_

#include <unordered_map>
#include <mutex>

#include "starq/types.hpp"
#include "starq/thread_runner.hpp"
#include "starq/leg_command_publisher.hpp"

namespace starq
{
    using Trajectory = std::vector<LegCommand::Ptr>;

    /// @brief Controller for executing a trajectory
    class TrajectoryController : public ThreadRunner
    {
    public:
        using Ptr = std::shared_ptr<TrajectoryController>;

        /// @brief Construct a new Trajectory Controller object
        /// @param leg_command_publisher Leg command publisher
        TrajectoryController(LegCommandPublisher::Ptr leg_command_publisher);

        /// @brief Destroy the Trajectory Controller object
        ~TrajectoryController();

        /// @brief Set the frequency of the trajectory
        /// @param frequency Frequency in Hz
        /// @return If the frequency was successfully set
        bool setFrequency(const Float frequency);

        /// @brief Set the number of loops for the trajectory
        /// @param num_loops Number of loops
        /// @return If the number of loops was successfully set
        bool setNumLoops(const size_t num_loops);

        /// @brief Set the trajectory
        /// @param trajectory Trajectory to set
        /// @return If the trajectory was successfully set
        bool setTrajectory(const Trajectory &trajectory);

        /// @brief Set the trajectory from a file
        /// @param file_path File path to load the trajectory from
        /// @return If the trajectory was successfully set
        bool setTrajectory(const std::string &file_path);

        /// @brief Get the frequency of the trajectory
        /// @return Frequency in Hz
        Float getFrequency();

        /// @brief Get the number of loops for the trajectory
        /// @return Number of loops
        size_t getNumLoops();

        /// @brief Get the trajectory
        /// @return Trajectory
        Trajectory getTrajectory();

    private:
        /// @brief Run the trajectory
        void run() override;

        /// @brief Load a trajectory from a file
        bool loadTrajectoryFromFile(const std::string &file_path, Trajectory &trajectory);

        /// @brief Sort the trajectory by time
        void sortTrajectory();

        LegCommandPublisher::Ptr leg_command_publisher_;

        Float frequency_;
        size_t num_loops_;
        Trajectory trajectory_;

        std::mutex mutex_;
        std::unordered_map<std::string, Trajectory> trajectory_file_cache_;
    };

}

#endif