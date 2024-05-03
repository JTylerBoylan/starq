#ifndef STARQ__TRAJECTORY_FILE_READER_HPP_
#define STARQ__TRAJECTORY_FILE_READER_HPP_

#include "starq/leg_command_publisher.hpp"

namespace starq
{

    /// @brief Reads leg trajectories from text files
    class TrajectoryFileReader
    {

    public:
        using Ptr = std::shared_ptr<TrajectoryFileReader>;

        /// @brief Create a trajectory file reader.
        TrajectoryFileReader();

        /// @brief Destroy the trajectory file reader.
        ~TrajectoryFileReader();

        /// @brief Load a 3D trajectory from a file.
        /// @param file_path Path to the file.
        /// @param frequency Frequency of the trajectory.
        /// @return True if the trajectory was loaded successfully, false otherwise.
        bool load(const std::string &file_path, const Float frequency = 1.0);

        /// @brief Get the trajectory.
        /// @return Vector of leg commands.
        std::vector<starq::LegCommand::Ptr> getTrajectory() { return trajectory_; }

    private:
        std::vector<starq::LegCommand::Ptr> trajectory_;
    };

}

#endif