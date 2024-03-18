#include "starq/operational_space_controller.hpp"

#include <iostream>

namespace starq
{

    OperationalSpaceController::OperationalSpaceController(const starq::LegDynamics::Ptr dynamics,
                                                           const std::vector<MotorController::Ptr> motor_controllers)
        : LegController(dynamics, motor_controllers)
    {
    }

    OperationalSpaceController::~OperationalSpaceController()
    {
    }

    bool OperationalSpaceController::setFootPosition(const Vector3 &foot_position,
                                                     const Vector3 &foot_velocity_ff,
                                                     const Vector3 &foot_accleration_ff)
    {
        const Vector3 current_joint_angles = getCurrentJointAngles();

        Matrix3 jacobian;
        if (!this->dynamics_->getJacobian(current_joint_angles, jacobian))
        {
            std::cerr << "Failed to get jacobian." << std::endl;
            return false;
        }

        const Vector3 pos_ref = foot_position;
        const Vector3 vel_ref = foot_velocity_ff;
        const Vector3 acc_ref = foot_accleration_ff;

        Vector3 pos_est, vel_est;
        getFootPositionEstimate(pos_est);
        getFootVelocityEstimate(vel_est);

        const Vector3 torque_ff = jacobian.transpose() * inertia_matrix_ * (acc_ref - d_inertia_matrix_dt_ * vel_ref) +
                                   coriolis_matrix_ * vel_ref + gravity_vector_;
        const Vector3 joint_torques = jacobian.transpose() * (kp_matrix_ * (pos_ref - pos_est) + kd_matrix_ * (vel_ref - vel_est)) +
                                       torque_ff;

        return setJointTorques(joint_torques);
    }

    void OperationalSpaceController::setInertiaMatrix(const Matrix3 &inertia_matrix)
    {
        inertia_matrix_ = inertia_matrix;
    }

    void OperationalSpaceController::setCoriolisMatrix(const Matrix3 &coriolis_matrix)
    {
        coriolis_matrix_ = coriolis_matrix;
    }

    void OperationalSpaceController::setGravityVector(const Vector3 &gravity_vector)
    {
        gravity_vector_ = gravity_vector;
    }

    void OperationalSpaceController::setKpMatrix(const Matrix3 &kp_matrix)
    {
        kp_matrix_ = kp_matrix;
    }

    void OperationalSpaceController::setKdMatrix(const Matrix3 &kd_matrix)
    {
        kd_matrix_ = kd_matrix;
    }

}