#ifndef STARQ__ROBOT_DYNAMICS_HPP_
#define STARQ__ROBOT_DYNAMICS_HPP_

#include <memory>
#include <vector>
#include "eigen3/Eigen/Dense"

namespace starq
{
    using namespace Eigen;

    class RobotDynamics
    {

    public:
        using Ptr = std::shared_ptr<RobotDynamics>;

        virtual float getBodyMass() const = 0;

        virtual Matrix3f getBodyInertia() const = 0;

        virtual Vector3f getGravity() const = 0;

        virtual float getBodyHeight() const = 0;

        virtual float getFootFriction() const = 0;

        virtual std::vector<Vector3f> getHipLocations() const = 0;

        virtual float getForceZMin() const = 0;

        virtual float getForceZMax() const = 0;
    };

}

#endif