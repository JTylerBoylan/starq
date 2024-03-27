#ifndef STARQ__ROBOT_PARAMETERS_HPP_
#define STARQ__ROBOT_PARAMETERS_HPP_

#include <memory>
#include <vector>
#include "starq/types.hpp"

namespace starq
{

    /// @brief RobotDynamics class
    class RobotParameters
    {

    public:
        using Ptr = std::shared_ptr<RobotParameters>;

        /// @brief Get the body mass
        /// @return The body mass
        virtual Float getBodyMass() const = 0;

        /// @brief Get the body inertia
        /// @return The body inertia
        virtual Matrix3 getBodyInertia() const = 0;

        /// @brief Get the gravity vector
        /// @return The gravity vector
        virtual Vector3 getGravity() const = 0;

        /// @brief Get the foot friction
        /// @return The foot friction
        virtual Float getFootFriction() const = 0;

        /// @brief Get the standing height
        /// @return The standing height
        virtual Float getStandingHeight() const = 0;

        /// @brief Get the hip locations
        /// @return The hip locations
        virtual std::vector<Vector3> getHipLocations() const = 0;

        /// @brief Get the default foot locations
        /// @return The default foot locations
        virtual std::vector<Vector3> getDefaultFootLocations() const = 0;

        /// @brief Get the minimum force in the z direction
        /// @return The minimum force in the z direction
        virtual Float getForceZMin() const = 0;

        /// @brief Get the maximum force in the z direction
        /// @return The maximum force in the z direction
        virtual Float getForceZMax() const = 0;
    };

}

#endif