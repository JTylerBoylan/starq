#ifndef STARQ__SYSTEM_LOCALIZATION_HPP_
#define STARQ__SYSTEM_LOCALIZATION_HPP_

#include "starq/slam/localization.hpp"

namespace starq
{
    using namespace slam;

    /// @brief System localization class
    class SystemLocalization : public Localization
    {
    public:
        using Ptr = std::shared_ptr<SystemLocalization>;

        /// @brief Construct a new System Localization object
        SystemLocalization() = default;

        /// @brief Destroy the System Localization object
        ~SystemLocalization() = default;

        /// @brief Get the current time.
        /// @return The current time in milliseconds.
        std::chrono::milliseconds getCurrentTime() override
        {
            return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
        
        }

        /// @brief Get the current position of the robot.
        /// @return Zero vector.
        Vector3f getCurrentPosition() override
        {
            return Vector3f::Zero();
        }

        /// @brief Get the current orientation of the robot.
        /// @return Zero vector.
        Vector3f getCurrentOrientation() override
        {
            return Vector3f::Zero();
        }

        /// @brief Get the current linear velocity of the robot.
        /// @return Zero vector.
        Vector3f getCurrentLinearVelocity() override
        {
            return Vector3f::Zero();
        }

        /// @brief Get the current angular velocity of the robot.
        /// @return Zero vector.
        Vector3f getCurrentAngularVelocity() override
        {
            return Vector3f::Zero();
        }
    };

}

#endif