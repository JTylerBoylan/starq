#include "starq/unitree/unitree_a1_robot_parameters.hpp"

namespace starq::unitree
{

    UnitreeA1RobotParameters::UnitreeA1RobotParameters()
    {
        body_mass_ = UNITREE_A1_MASS;
        body_inertia_ << UNITREE_A1_INERTIA;
        gravity_ << UNITREE_A1_GRAVITY;
        foot_friction_ = UNITREE_A1_FOOT_FRICTION;
        standing_height_ = UNITREE_A1_STAND_HEIGHT;
        hip_locations_ = {Vector3(UNITREE_A1_HIP_OFFSET_X, UNITREE_A1_HIP_OFFSET_Y, 0.0),
                          Vector3(-UNITREE_A1_HIP_OFFSET_X, UNITREE_A1_HIP_OFFSET_Y, 0.0),
                          Vector3(-UNITREE_A1_HIP_OFFSET_X, -UNITREE_A1_HIP_OFFSET_Y, 0.0),
                          Vector3(UNITREE_A1_HIP_OFFSET_X, -UNITREE_A1_HIP_OFFSET_Y, 0.0)};
        default_foot_locations_ = {Vector3(0.0, UNITREE_A1_LENGTH_D, -UNITREE_A1_STAND_HEIGHT),
                                   Vector3(0.0, UNITREE_A1_LENGTH_D, -UNITREE_A1_STAND_HEIGHT),
                                   Vector3(0.0, -UNITREE_A1_LENGTH_D, -UNITREE_A1_STAND_HEIGHT),
                                   Vector3(0.0, -UNITREE_A1_LENGTH_D, -UNITREE_A1_STAND_HEIGHT)};
        force_z_min_ = UNITREE_A1_Z_FORCE_MIN;
        force_z_max_ = UNITREE_A1_Z_FORCE_MAX;
    }

    UnitreeA1RobotParameters::~UnitreeA1RobotParameters()
    {
    }

    Float UnitreeA1RobotParameters::getBodyMass() const
    {
        return body_mass_;
    }

    Matrix3 UnitreeA1RobotParameters::getBodyInertia() const
    {
        return body_inertia_;
    }

    Vector3 UnitreeA1RobotParameters::getGravity() const
    {
        return gravity_;
    }

    Float UnitreeA1RobotParameters::getFootFriction() const
    {
        return foot_friction_;
    }

    Float UnitreeA1RobotParameters::getStandingHeight() const
    {
        return standing_height_;
    }

    std::vector<Vector3> UnitreeA1RobotParameters::getHipLocations() const
    {
        return hip_locations_;
    }

    std::vector<Vector3> UnitreeA1RobotParameters::getDefaultFootLocations() const
    {
        return default_foot_locations_;
    }

    Float UnitreeA1RobotParameters::getForceZMin() const
    {
        return force_z_min_;
    }

    Float UnitreeA1RobotParameters::getForceZMax() const
    {
        return force_z_max_;
    }

}