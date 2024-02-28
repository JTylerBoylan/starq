#ifndef STARQ_MUJOCO__MUJOCO_LOCALIZATION_HPP_
#define STARQ_MUJOCO__MUJOCO_LOCALIZATION_HPP_

#include "starq/mujoco/mujoco.hpp"
#include "starq/slam/localization.hpp"

namespace starq::mujoco
{

    class MuJoCoLocalization : public slam::Localization
    {
    public:
        using Ptr = std::shared_ptr<MuJoCoLocalization>;

        /// @brief Constructor
        MuJoCoLocalization(MuJoCo::Ptr mujoco);

        /// @brief Get the current time.
        /// @return The current time in milliseconds.
        virtual std::chrono::milliseconds getCurrentTime() override;

        /// @brief Get the current position of the robot.
        /// @return Position vector [m] (x, y, z) in the world frame.
        virtual Eigen::Vector3f getCurrentPosition() override;

        /// @brief Get the current orientation of the robot.
        /// @return Orientation vector [rad] (roll, pitch, yaw) in the world frame.
        virtual Eigen::Vector3f getCurrentOrientation() override;

        /// @brief Get the current linear velocity of the robot.
        /// @return Velocity vector [m/s] (x, y, z) in the world frame.
        virtual Eigen::Vector3f getCurrentLinearVelocity() override;

        /// @brief Get the current angular velocity of the robot.
        /// @return Angular velocity vector [rad/s] (roll, pitch, yaw) in the world frame.
        virtual Eigen::Vector3f getCurrentAngularVelocity() override;

    private:

        /// @brief Get state information
        void controlMotor(const mjModel *model, mjData *data);

        /// @brief Convert quaternion to euler angles
        void quat2eul(const Eigen::Quaternionf &q, Eigen::Vector3f &eul);

        Eigen::Vector3f position_;
        Eigen::Vector3f orientation_;
        Eigen::Vector3f linear_velocity_;
        Eigen::Vector3f angular_velocity_;

        Eigen::Vector3f last_position_;
        Eigen::Vector3f last_orientation_;
        mjtNum last_time_;

    };

}

#endif