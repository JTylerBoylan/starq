#ifndef STARQ_MUJOCO__MUJOCO_ROBOT_HPP_
#define STARQ_MUJOCO__MUJOCO_ROBOT_HPP_

#include <future>

#include "starq/mujoco/mujoco.hpp"
#include "starq/mujoco/mujoco_controller.hpp"
#include "starq/mujoco/mujoco_localization.hpp"
#include "starq/robot.hpp"

namespace starq::mujoco
{

    /// @brief MuJoCo robot class
    class MuJoCoRobot : public Robot
    {
    public:
        using Ptr = std::shared_ptr<MuJoCoRobot>;

        /// @brief Constructor
        MuJoCoRobot();

        /// @brief Set the scene file
        /// @param scene_file The scene file
        void setSceneFile(const std::string &scene_file);

        /// @brief Start the simulation
        void startSimulation();

        /// @brief Wait for the simulation to finish
        void waitForSimulation();

        /// @brief Check if the simulation is open
        /// @return True if open
        bool isSimulationOpen();

    protected:

        /// @brief Setup the localization
        void setupLocalization() override;

        MuJoCo::Ptr mujoco_;

        std::string scene_file_;
        std::future<void> simulation_;
    };

}

#endif