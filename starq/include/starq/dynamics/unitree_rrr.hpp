#ifndef STARQ_DYNAMICS__UNITREE_RRR_HPP_
#define STARQ_DYNAMICS__UNITREE_RRR_HPP_

#include "starq/leg_dynamics.hpp"

namespace starq::dynamics
{

    /// @brief Implementation of LegDynamics for the Unitree RRR leg
    class Unitree_RRR : public LegDynamics
    {
    public:
        using Ptr = std::shared_ptr<Unitree_RRR>;

        /// @brief Constructor for RRR leg.
        /// @param d Abduction link length.
        /// @param lt Thigh link length.
        /// @param lc Calf link length.
        Unitree_RRR(float d, float lt, float lc, int a_axis);

        /// @brief Forward kinematics for FiveBar2D leg.
        /// @param joint_angles Joint angles.
        /// @param foot_position Foot position.
        /// @return If the forward kinematics was successful.
        bool getForwardKinematics(const VectorXf &joint_angles, VectorXf &foot_position) override;

        /// @brief Inverse kinematics for FiveBar2D leg.
        /// @param foot_position Foot position.
        /// @param joint_angles Joint angles.
        /// @return If the inverse kinematics was successful.
        bool getInverseKinematics(const VectorXf &foot_position, VectorXf &joint_angles) override;

        /// @brief Jacobian for FiveBar2D leg.
        /// @param joint_angles Joint angles.
        /// @param jacobian Jacobian matrix.
        /// @return If the Jacobian matrix was successful.
        bool getJacobian(const VectorXf &joint_angles, MatrixXf &jacobian) override;

    private:
        float d_, lt_, lc_, a_axis_;
    };

}

#endif