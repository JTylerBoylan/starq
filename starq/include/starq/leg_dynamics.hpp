#ifndef STARQ__LEG_DYNAMICS_HPP_
#define STARQ__LEG_DYNAMICS_HPP_

#include <memory>
#include "eigen3/Eigen/Dense"

namespace starq
{
    using namespace Eigen;

    /// @brief Abstract class for different leg dynamic types
    class LegDynamics
    {
    public:
        using Ptr = std::shared_ptr<LegDynamics>;

        /// @brief Get the forward kinematics of a set of joint angles.
        /// @param joint_angles Joint angles.
        /// @param foot_position Foot position.
        /// @return If the forward kinematics was successful.
        virtual bool getForwardKinematics(const VectorXf &joint_angles, VectorXf &foot_position) = 0;

        /// @brief Get the inverse kinematics of a foot position.
        /// @param foot_position Foot position.
        /// @param joint_angles Joint angles.
        /// @return If the inverse kinematics was successful.
        virtual bool getInverseKinematics(const VectorXf &foot_position, VectorXf &joint_angles) = 0;

        /// @brief Get the Jacobian matrix for a set of joint angles.
        /// @param joint_angles Joint angles.
        /// @param jacobian Jacobian matrix.
        /// @return If the Jacobian matrix was successful.
        virtual bool getJacobian(const VectorXf &joint_angles, MatrixXf &jacobian) = 0;
        
    };
}

#endif