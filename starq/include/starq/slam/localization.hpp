#ifndef STARQ_SLAM__LOCALIZATION_HPP_
#define STARQ_SLAM__LOCALIZATION_HPP_

#include <memory>
#include <chrono>

#include "eigen3/Eigen/Dense"

namespace starq::slam
{
    using namespace Eigen;

    class Localization
    {
    public:
        using Ptr = std::shared_ptr<Localization>;

        /// @brief Get the current time.
        /// @return The current time in milliseconds.
        virtual std::chrono::milliseconds getCurrentTime() = 0;

        /// @brief Get the current position of the robot.
        /// @return Position vector [m] (x, y, z) in the world frame.
        virtual Vector3f getCurrentPosition() = 0;

        /// @brief Get the current orientation of the robot.
        /// @return Orientation vector [rad] (roll, pitch, yaw) in the world frame.
        virtual Vector3f getCurrentOrientation() = 0;

        /// @brief Get the current linear velocity of the robot.
        /// @return Velocity vector [m/s] (x, y, z) in the world frame.
        virtual Vector3f getCurrentLinearVelocity() = 0;

        /// @brief Get the current angular velocity of the robot.
        /// @return Angular velocity vector [rad/s] (roll, pitch, yaw) in the world frame.
        virtual Vector3f getCurrentAngularVelocity() = 0;

    private:
    };
}

#endif