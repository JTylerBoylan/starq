#include "starq/leg_controller.hpp"
#include <iostream>

namespace starq
{

    LegController::LegController(const starq::LegDynamics::Ptr dynamics,
                                 const std::vector<MotorController::Ptr> motor_controllers)
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

    bool LegController::setFootPosition(const VectorXf &foot_position,
                                        const VectorXf &foot_velocity_ff,
                                        const VectorXf &foot_torque_ff)
    {

        if (foot_position.size() == 0)
        {
            std::cerr << "Empty foot position." << std::endl;
            return false;
        }

        const bool has_velocity_ff = foot_velocity_ff.size() > 0;
        const bool has_torque_ff = foot_torque_ff.size() > 0;

        const VectorXf current_joint_angles = getCurrentJointAngles();

        MatrixXf jacobian;
        if ((has_velocity_ff || has_torque_ff) &&
            !dynamics_->getJacobian(current_joint_angles, jacobian))
        {
            std::cerr << "Failed to get Jacobian." << std::endl;
            return false;
        }

        VectorXf joint_angles;
        if (!dynamics_->getInverseKinematics(foot_position, joint_angles))
        {
            std::cerr << "Failed to get inverse kinematics." << std::endl;
            return false;
        }

        const VectorXf joint_velocity = has_velocity_ff ? (VectorXf)(jacobian.inverse() * foot_velocity_ff)
                                                        : VectorXf::Zero(joint_angles.size());

        const VectorXf joint_torque = has_torque_ff ? (VectorXf)(jacobian.transpose() * foot_torque_ff)
                                                    : VectorXf::Zero(joint_angles.size());

        return setJointAngles(joint_angles, joint_velocity, joint_torque);
    }

    bool LegController::setFootVelocity(const VectorXf &foot_velocity,
                                        const VectorXf &foot_torque_ff)
    {

        if (foot_velocity.size() == 0)
        {
            std::cerr << "Empty foot velocity." << std::endl;
            return false;
        }

        const bool has_torque_ff = foot_torque_ff.size() > 0;

        const VectorXf current_joint_angles = getCurrentJointAngles();

        MatrixXf jacobian;
        if (!dynamics_->getJacobian(current_joint_angles, jacobian))
        {
            std::cerr << "Failed to get Jacobian." << std::endl;
            return false;
        }

        const VectorXf joint_velocities = jacobian.inverse() * foot_velocity;

        const VectorXf joint_torque = has_torque_ff ? (VectorXf)(jacobian.transpose() * foot_torque_ff)
                                                    : VectorXf::Zero(joint_velocities.size());

        return setJointVelocities(joint_velocities, joint_torque);
    }

    bool LegController::setFootForce(const VectorXf &foot_force)
    {

        if (foot_force.size() == 0)
        {
            std::cerr << "Empty foot force." << std::endl;
            return false;
        }

        const VectorXf current_joint_angles = getCurrentJointAngles();

        MatrixXf jacobian;
        if (!dynamics_->getJacobian(current_joint_angles, jacobian))
        {
            std::cerr << "Failed to get Jacobian." << std::endl;
            return false;
        }

        const VectorXf joint_torque = jacobian.transpose() * foot_force;

        return setJointTorques(joint_torque);
    }

    bool LegController::getFootPositionEstimate(VectorXf &foot_position)
    {

        const VectorXf current_joint_angles = getCurrentJointAngles();

        if (!dynamics_->getForwardKinematics(current_joint_angles, foot_position))
        {
            std::cerr << "Failed to get forward kinematics." << std::endl;
            return false;
        }

        return true;
    }

    bool LegController::getFootVelocityEstimate(VectorXf &foot_velocity)
    {

        const VectorXf current_joint_angles = getCurrentJointAngles();

        MatrixXf jacobian;
        if (!dynamics_->getJacobian(current_joint_angles, jacobian))
        {
            std::cerr << "Failed to get Jacobian." << std::endl;
            return false;
        }

        const VectorXf joint_velocities = getCurrentJointVelocities();
    
        foot_velocity = jacobian * joint_velocities;

        return true;
    }

    bool LegController::getFootForceEstimate(VectorXf &foot_force)
    {

        const VectorXf current_joint_angles = getCurrentJointAngles();

        MatrixXf jacobian;
        if (!dynamics_->getJacobian(current_joint_angles, jacobian))
        {
            std::cerr << "Failed to get Jacobian." << std::endl;
            return false;
        }

        const VectorXf joint_torques = getCurrentJointTorques();
        
        foot_force = jacobian.transpose() * joint_torques;

        return true;
    }

    VectorXf LegController::getCurrentJointAngles()
    {
        VectorXf joint_angles(motor_controllers_.size());
        for (size_t i = 0; i < motor_controllers_.size(); i++)
        {
            joint_angles(i) = motor_controllers_[i]->getPositionEstimate();
        }
        return joint_angles;
    }

    VectorXf LegController::getCurrentJointVelocities()
    {
        VectorXf joint_velocities(motor_controllers_.size());
        for (size_t i = 0; i < motor_controllers_.size(); i++)
        {
            joint_velocities(i) = motor_controllers_[i]->getVelocityEstimate();
        }
        return joint_velocities;
    }

    VectorXf LegController::getCurrentJointTorques()
    {
        VectorXf joint_torques(motor_controllers_.size());
        for (size_t i = 0; i < motor_controllers_.size(); i++)
        {
            joint_torques(i) = motor_controllers_[i]->getTorqueEstimate();
        }
        return joint_torques;
    }

    bool LegController::setJointAngles(const VectorXf &joint_angles,
                                       const VectorXf &joint_velocity_ff,
                                       const VectorXf &joint_torque_ff)
    {
        bool success = true;
        for (size_t i = 0; (i < motor_controllers_.size()) && success; i++)
        {
            success &= motor_controllers_[i]->setPosition(joint_angles(i, 0),
                                                          joint_velocity_ff(i, 0),
                                                          joint_torque_ff(i, 0));
        }
        return success;
    }

    bool LegController::setJointVelocities(const VectorXf &joint_velocities,
                                           const VectorXf &joint_torque_ff)
    {
        bool success = true;
        for (size_t i = 0; (i < motor_controllers_.size()) && success; i++)
        {
            success &= motor_controllers_[i]->setVelocity(joint_velocities(i, 0),
                                                          joint_torque_ff(i, 0));
        }
        return success;
    }

    bool LegController::setJointTorques(const VectorXf &joint_torques)
    {
        bool success = true;
        for (size_t i = 0; (i < motor_controllers_.size()) && success; i++)
        {
            success &= motor_controllers_[i]->setTorque(joint_torques(i, 0));
        }
        return success;
    }

}