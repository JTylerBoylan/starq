#include "starq/leg_controller.hpp"
#include <iostream>

namespace starq
{

    LegController::LegController(const starq::LegDynamics::Ptr dynamics,
                                 const MotorList motor_controllers)
        : dynamics_(dynamics),
          motor_controllers_(motor_controllers)
    {
    }

    LegController::~LegController()
    {
    }

    bool LegController::setState(const uint32_t state)
    {

        bool success = true;
        for (const auto &motor : motor_controllers_)
        {
            success &= motor->setState(state);
        }

        return success;
    }

    bool LegController::setControlMode(const uint32_t control_mode, const uint32_t input_mode)
    {

        bool success = true;
        for (const auto &motor : motor_controllers_)
        {
            success &= motor->setControlMode(control_mode, input_mode);
        }

        return success;
    }

    bool LegController::setFootPosition(const Vector3 &foot_position,
                                        const Vector3 &foot_velocity_ff,
                                        const Vector3 &foot_torque_ff)
    {
        const Vector3 current_joint_angles = getCurrentJointAngles();

        Matrix3 jacobian;
        if (!dynamics_->getJacobian(current_joint_angles, jacobian))
        {
            std::cerr << "Failed to get Jacobian." << std::endl;
            return false;
        }

        Vector3 joint_angles;
        if (!dynamics_->getInverseKinematics(foot_position, joint_angles))
        {
            std::cerr << "Failed to get inverse kinematics." << std::endl;
            return false;
        }

        const Vector3 joint_velocity = jacobian.inverse() * foot_velocity_ff;
        const Vector3 joint_torque = jacobian.transpose() * foot_torque_ff;

        return setJointAngles(joint_angles, joint_velocity, joint_torque);
    }

    bool LegController::setFootVelocity(const Vector3 &foot_velocity,
                                        const Vector3 &foot_torque_ff)
    {
        const Vector3 current_joint_angles = getCurrentJointAngles();

        Matrix3 jacobian;
        if (!dynamics_->getJacobian(current_joint_angles, jacobian))
        {
            std::cerr << "Failed to get Jacobian." << std::endl;
            return false;
        }

        const Vector3 joint_velocities = jacobian.inverse() * foot_velocity;
        const Vector3 joint_torque = jacobian.transpose() * foot_torque_ff;

        return setJointVelocities(joint_velocities, joint_torque);
    }

    bool LegController::setFootForce(const Vector3 &foot_force)
    {
        const Vector3 current_joint_angles = getCurrentJointAngles();

        Matrix3 jacobian;
        if (!dynamics_->getJacobian(current_joint_angles, jacobian))
        {
            std::cerr << "Failed to get Jacobian." << std::endl;
            return false;
        }

        const Vector3 joint_torque = jacobian.transpose() * foot_force;

        return setJointTorques(joint_torque);
    }

    bool LegController::getFootPositionEstimate(Vector3 &foot_position)
    {

        const Vector3 current_joint_angles = getCurrentJointAngles();

        if (!dynamics_->getForwardKinematics(current_joint_angles, foot_position))
        {
            std::cerr << "Failed to get forward kinematics." << std::endl;
            return false;
        }

        return true;
    }

    bool LegController::getFootVelocityEstimate(Vector3 &foot_velocity)
    {

        const Vector3 current_joint_angles = getCurrentJointAngles();

        Matrix3 jacobian;
        if (!dynamics_->getJacobian(current_joint_angles, jacobian))
        {
            std::cerr << "Failed to get Jacobian." << std::endl;
            return false;
        }

        const Vector3 joint_velocities = getCurrentJointVelocities();
    
        foot_velocity = jacobian * joint_velocities;

        return true;
    }

    bool LegController::getFootForceEstimate(Vector3 &foot_force)
    {

        const Vector3 current_joint_angles = getCurrentJointAngles();

        Matrix3 jacobian;
        if (!dynamics_->getJacobian(current_joint_angles, jacobian))
        {
            std::cerr << "Failed to get Jacobian." << std::endl;
            return false;
        }

        const Vector3 joint_torques = getCurrentJointTorques();
        
        foot_force = jacobian.transpose() * joint_torques;

        return true;
    }

    Vector3 LegController::getCurrentJointAngles()
    {
        Vector3 joint_angles = Vector3::Zero();
        for (size_t i = 0; i < motor_controllers_.size(); i++)
        {
            joint_angles(i) = motor_controllers_[i]->getPositionEstimate();
        }
        return joint_angles;
    }

    Vector3 LegController::getCurrentJointVelocities()
    {
        Vector3 joint_velocities = Vector3::Zero();
        for (size_t i = 0; i < motor_controllers_.size(); i++)
        {
            joint_velocities(i) = motor_controllers_[i]->getVelocityEstimate();
        }
        return joint_velocities;
    }

    Vector3 LegController::getCurrentJointTorques()
    {
        Vector3 joint_torques = Vector3::Zero();
        for (size_t i = 0; i < motor_controllers_.size(); i++)
        {
            joint_torques(i) = motor_controllers_[i]->getTorqueEstimate();
        }
        return joint_torques;
    }

    bool LegController::setJointAngles(const Vector3 &joint_angles,
                                       const Vector3 &joint_velocity_ff,
                                       const Vector3 &joint_torque_ff)
    {
        bool success = true;
        for (size_t i = 0; (i < motor_controllers_.size()) && success; i++)
        {
            const Float angle = joint_angles(i, 0);
            const Float velocity_ff = joint_velocity_ff(i, 0);
            const Float torque_ff = joint_torque_ff(i, 0);
            success &= motor_controllers_[i]->setPosition(angle,
                                                          velocity_ff,
                                                          torque_ff);
        }
        return success;
    }

    bool LegController::setJointVelocities(const Vector3 &joint_velocities,
                                           const Vector3 &joint_torque_ff)
    {
        bool success = true;
        for (size_t i = 0; (i < motor_controllers_.size()) && success; i++)
        {
            const Float velocity = joint_velocities(i, 0);
            const Float torque_ff = joint_torque_ff(i, 0);
            success &= motor_controllers_[i]->setVelocity(velocity,
                                                          torque_ff);
        }
        return success;
    }

    bool LegController::setJointTorques(const Vector3 &joint_torques)
    {
        bool success = true;
        for (size_t i = 0; (i < motor_controllers_.size()) && success; i++)
        {
            const Float torque = joint_torques(i, 0);
            success &= motor_controllers_[i]->setTorque(torque);
        }
        return success;
    }

}