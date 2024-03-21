#ifndef STARQ_SLAM__LOCALIZATION_HPP_
#define STARQ_SLAM__LOCALIZATION_HPP_

#include <memory>
#include "starq/types.hpp"

namespace starq::slam
{

    class Localization
    {
    public:
        using Ptr = std::shared_ptr<Localization>;

        /// @brief Get the current time.
        /// @return The current time in milliseconds.
        virtual milliseconds getCurrentTime() = 0;

        /// @brief Get the current position of the robot.
        /// @return Position vector [m] (x, y, z) in the world frame.
        virtual Vector3 getCurrentPosition() = 0;

        /// @brief Get the current orientation of the robot.
        /// @return Orientation vector [rad] (roll, pitch, yaw) in the world frame.
        virtual Vector3 getCurrentOrientation() = 0;

        /// @brief Get the current linear velocity of the robot.
        /// @return Velocity vector [m/s] (x, y, z) in the world frame.
        virtual Vector3 getCurrentLinearVelocity() = 0;

        /// @brief Get the current angular velocity of the robot.
        /// @return Angular velocity vector [rad/s] (roll, pitch, yaw) in the world frame.
        virtual Vector3 getCurrentAngularVelocity() = 0;

        /// @brief Convert orientation vector to rotation matrix.
        /// @return The rotation matrix.
        inline Matrix3 toRotationMatrix(const Vector3 &orientation)
        {
            Matrix3 rotation;
            rotation = Eigen::AngleAxis<Float>(orientation.z(), Vector3::UnitZ()) *
                       Eigen::AngleAxis<Float>(orientation.y(), Vector3::UnitY()) *
                       Eigen::AngleAxis<Float>(orientation.x(), Vector3::UnitX());
            return rotation;
        }

    private:
    };
}

#endif