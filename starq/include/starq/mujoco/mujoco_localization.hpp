#ifndef STARQ_MUJOCO__MUJOCO_LOCALIZATION_HPP_
#define STARQ_MUJOCO__MUJOCO_LOCALIZATION_HPP_

#include "starq/mujoco/mujoco.hpp"
#include "starq/slam/localization.hpp"

namespace starq::mujoco
{

    /// @brief MuJoCo localization class
    class MuJoCoLocalization : public slam::Localization
    {
    public:
        using Ptr = std::shared_ptr<MuJoCoLocalization>;

        /// @brief Constructor
        MuJoCoLocalization(MuJoCo::Ptr mujoco);

        /// @brief Get the current time.
        /// @return The current time in milliseconds.
        virtual milliseconds getCurrentTime() override;

        /// @brief Get the current position of the robot.
        /// @return Position vector [m] (x, y, z) in the world frame.
        virtual Vector3 getCurrentPosition() override;

        /// @brief Get the current orientation of the robot.
        /// @return Orientation vector [rad] (roll, pitch, yaw) in the world frame.
        virtual Vector3 getCurrentOrientation() override;

        /// @brief Get the current linear velocity of the robot.
        /// @return Velocity vector [m/s] (x, y, z) in the world frame.
        virtual Vector3 getCurrentLinearVelocity() override;

        /// @brief Get the current angular velocity of the robot.
        /// @return Angular velocity vector [rad/s] (roll, pitch, yaw) in the world frame.
        virtual Vector3 getCurrentAngularVelocity() override;

    private:
        /// @brief Get state information
        void localizationCallback(const mjModel *model, mjData *data);

        /// @brief Convert quaternion to euler angles
        void quat2eul(const Eigen::Quaternion<Float> &q, Vector3 &eul);

        Vector3 position_;
        Vector3 orientation_;
        Vector3 linear_velocity_;
        Vector3 angular_velocity_;

        Vector3 last_position_;
        Vector3 last_orientation_;
        mjtNum last_time_;
    };

}

#endif