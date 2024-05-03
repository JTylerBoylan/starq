#include "starq/starq/starq_robot_parameters.hpp"

namespace starq
{

    STARQRobotParameters::STARQRobotParameters()
    {
        body_mass_ = STARQ_MASS;
        body_inertia_ << STARQ_INERTIA;
        gravity_ << STARQ_GRAVITY;
        foot_friction_ = STARQ_FOOT_FRICTION;
        standing_height_ = STARQ_STAND_HEIGHT;
        hip_locations_ = {Vector3(STARQ_HIP_OFFSET_X, STARQ_HIP_OFFSET_Y, 0.0),
                          Vector3(-STARQ_HIP_OFFSET_X, STARQ_HIP_OFFSET_Y, 0.0),
                          Vector3(-STARQ_HIP_OFFSET_X, -STARQ_HIP_OFFSET_Y, 0.0),
                          Vector3(STARQ_HIP_OFFSET_X, -STARQ_HIP_OFFSET_Y, 0.0)};
        default_foot_locations_ = {Vector3(0.0, 0.0, -STARQ_STAND_HEIGHT),
                                   Vector3(0.0, 0.0, -STARQ_STAND_HEIGHT),
                                   Vector3(0.0, 0.0, -STARQ_STAND_HEIGHT),
                                   Vector3(0.0, 0.0, -STARQ_STAND_HEIGHT)};
        force_z_min_ = STARQ_Z_FORCE_MIN;
        force_z_max_ = STARQ_Z_FORCE_MAX;
        is_2d_ = STARQ_IS_2D;
    }

    Float STARQRobotParameters::getBodyMass() const
    {
        return body_mass_;
    }

    Matrix3 STARQRobotParameters::getBodyInertia() const
    {
        return body_inertia_;
    }

    Vector3 STARQRobotParameters::getGravity() const
    {
        return gravity_;
    }

    Float STARQRobotParameters::getFootFriction() const
    {
        return foot_friction_;
    }

    Float STARQRobotParameters::getStandingHeight() const
    {
        return standing_height_;
    }

    std::vector<Vector3> STARQRobotParameters::getHipLocations() const
    {
        return hip_locations_;
    }

    std::vector<Vector3> STARQRobotParameters::getDefaultFootLocations() const
    {
        return default_foot_locations_;
    }

    Float STARQRobotParameters::getForceZMin() const
    {
        return force_z_min_;
    }

    Float STARQRobotParameters::getForceZMax() const
    {
        return force_z_max_;
    }

    bool STARQRobotParameters::is2D() const
    {
        return is_2d_;
    }

}