#ifndef STARQ_ROBOTS__UNITREE_A1_MUJOCO_HPP_
#define STARQ_ROBOTS__UNITREE_A1_MUJOCO_HPP_

#include <future>

#include "starq/mujoco/mujoco_controller.hpp"
#include "starq/mujoco/mujoco_localization.hpp"
#include "starq/dynamics/unitree_rrr.hpp"
#include "starq/leg_controller.hpp"
#include "starq/leg_command_publisher.hpp"
#include "starq/trajectory_file_reader.hpp"
#include "starq/trajectory_publisher.hpp"

#define UNITREE_A1_LENGTH_D 0.08505
#define UNITREE_A1_LENGTH_LT 0.2
#define UNITREE_A1_LENGTH_LC 0.2

namespace starq::robots
{
    using namespace starq::mujoco;
    using namespace starq::dynamics;

    class UnitreeA1MuJoCoRobot
    {
    public:
        enum LegId
        {
            FR = 0,
            FL = 1,
            RR = 2,
            RL = 3
        };

        using Ptr = std::shared_ptr<UnitreeA1MuJoCoRobot>;

        UnitreeA1MuJoCoRobot();

        std::vector<MuJoCoController::Ptr> getMotors() const { return motors_; }

        Unitree_RRR::Ptr getLeftLegDynamics() const { return unitree_RRR_L_; }

        Unitree_RRR::Ptr getRightLegDynamics() const { return unitree_RRR_R_; }

        std::vector<LegController::Ptr> getLegs() const { return legs_; }

        slam::Localization::Ptr getLocalization() const { return localization_; }

        LegCommandPublisher::Ptr getLegCommandPublisher() const { return publisher_; }

        TrajectoryFileReader::Ptr getTrajectoryFileReader() const { return trajectory_file_reader_; }

        TrajectoryPublisher::Ptr getTrajectoryPublisher() const { return trajectory_publisher_; }

        void setSceneFile(const std::string &scene_file) { scene_file_ = scene_file; }

        void startSimulation();

        void waitForSimulation();

        bool isSimulationOpen();

    private:
        MuJoCo::Ptr mujoco_;

        std::vector<MuJoCoController::Ptr> motors_;
        Unitree_RRR::Ptr unitree_RRR_L_;
        Unitree_RRR::Ptr unitree_RRR_R_;
        std::vector<LegController::Ptr> legs_;
        slam::Localization::Ptr localization_;
        LegCommandPublisher::Ptr publisher_;
        TrajectoryFileReader::Ptr trajectory_file_reader_;
        TrajectoryPublisher::Ptr trajectory_publisher_;

        std::string scene_file_;
        std::future<void> simulation_;
    };

}

#endif