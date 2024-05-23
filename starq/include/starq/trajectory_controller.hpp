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

    class TrajectoryController : public ThreadRunner
    {
    public:
        using Ptr = std::shared_ptr<TrajectoryController>;

        TrajectoryController(LegCommandPublisher::Ptr leg_command_publisher);

        bool setFrequency(const Float frequency);

        bool setNumLoops(const size_t num_loops);

        bool setTrajectory(const Trajectory &trajectory);

        bool setTrajectory(const std::string &file_path);

        Float getFrequency();

        size_t getNumLoops();

        Trajectory getTrajectory();

    private:
        void run() override;

        bool loadTrajectoryFromFile(const std::string &file_path, Trajectory &trajectory);

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