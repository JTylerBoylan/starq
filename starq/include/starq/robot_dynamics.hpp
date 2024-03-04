#ifndef STARQ__ROBOT_DYNAMICS_HPP_
#define STARQ__ROBOT_DYNAMICS_HPP_

#include <memory>
#include <vector>
#include "eigen3/Eigen/Dense"

namespace starq
{
    using namespace Eigen;

    /// @brief RobotDynamics class
    class RobotDynamics
    {

    public:
        using Ptr = std::shared_ptr<RobotDynamics>;

        /// @brief Get the body mass
        /// @return The body mass
        virtual float getBodyMass() const = 0;

        /// @brief Get the body inertia
        /// @return The body inertia
        virtual Matrix3f getBodyInertia() const = 0;

        /// @brief Get the gravity vector
        /// @return The gravity vector
        virtual Vector3f getGravity() const = 0;

        /// @brief Get the body height
        /// @return The body height
        virtual float getBodyHeight() const = 0;

        /// @brief Get the foot friction
        /// @return The foot friction
        virtual float getFootFriction() const = 0;

        /// @brief Get the hip locations
        /// @return The hip locations
        virtual std::vector<Vector3f> getHipLocations() const = 0;

        /// @brief Get the minimum force in the z direction
        /// @return The minimum force in the z direction
        virtual float getForceZMin() const = 0;

        /// @brief Get the maximum force in the z direction
        /// @return The maximum force in the z direction
        virtual float getForceZMax() const = 0;
    };

}

#endif