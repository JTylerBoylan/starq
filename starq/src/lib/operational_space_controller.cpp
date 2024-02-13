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

    bool OperationalSpaceController::setFootPosition(const VectorXf &foot_position,
                                                     const VectorXf &foot_velocity_ff,
                                                     const VectorXf &foot_accleration_ff)
    {

        if (foot_position.size() == 0)
        {
            std::cerr << "Empty foot position." << std::endl;
            return false;
        }

        const bool has_velocity_ff = foot_velocity_ff.size() > 0;
        const bool has_accleration_ff = foot_accleration_ff.size() > 0;

        const VectorXf current_joint_angles = getCurrentJointAngles();

        MatrixXf jacobian;
        if ((has_velocity_ff || has_accleration_ff) &&
            !this->dynamics_->getJacobian(current_joint_angles, jacobian))
        {
            std::cerr << "Failed to get jacobian." << std::endl;
            return false;
        }

        const VectorXf pos_ref = foot_position;
        const VectorXf vel_ref = has_velocity_ff ? foot_velocity_ff : VectorXf::Zero(foot_position.size());
        const VectorXf acc_ref = has_accleration_ff ? foot_accleration_ff : VectorXf::Zero(foot_position.size());

        VectorXf pos_est, vel_est;
        getFootPositionEstimate(pos_est);
        getFootVelocityEstimate(vel_est);

        const VectorXf torque_ff = jacobian.transpose() * inertia_matrix_ * (acc_ref - d_inertia_matrix_dt_ * vel_ref) +
                                   coriolis_matrix_ * vel_ref + gravity_vector_;
        const VectorXf joint_torques = jacobian.transpose() * (kp_matrix_ * (pos_ref - pos_est) + kd_matrix_ * (vel_ref - vel_est)) +
                                       torque_ff;

        return setJointTorques(joint_torques);
    }

    void OperationalSpaceController::setInertiaMatrix(const MatrixXf &inertia_matrix)
    {
        inertia_matrix_ = inertia_matrix;
    }

    void OperationalSpaceController::setCoriolisMatrix(const MatrixXf &coriolis_matrix)
    {
        coriolis_matrix_ = coriolis_matrix;
    }

    void OperationalSpaceController::setGravityVector(const VectorXf &gravity_vector)
    {
        gravity_vector_ = gravity_vector;
    }

    void OperationalSpaceController::setKpMatrix(const MatrixXf &kp_matrix)
    {
        kp_matrix_ = kp_matrix;
    }

    void OperationalSpaceController::setKdMatrix(const MatrixXf &kd_matrix)
    {
        kd_matrix_ = kd_matrix;
    }

}