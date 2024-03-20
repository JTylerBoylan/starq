#ifndef STARQ_DYNAMICS__UNITREE_A1_LEG_DYNAMICS_HPP_
#define STARQ_DYNAMICS__UNITREE_A1_LEG_DYNAMICS_HPP_

#include "starq/leg_dynamics.hpp"

namespace starq::unitree
{

    /// @brief Implementation of LegDynamics for the Unitree RRR leg
    class UnitreeA1LegDynamics : public LegDynamics
    {
    public:
        using Ptr = std::shared_ptr<UnitreeA1LegDynamics>;

        /// @brief Constructor for RRR leg.
        /// @param d Abduction link length.
        /// @param lt Thigh link length.
        UnitreeA1LegDynamics(Float d, Float lt, Float lc);

        /// @brief Forward kinematics for FiveBar2D leg.
        /// @param joint_angles Joint angles.
        /// @param foot_position Foot position.
        /// @return If the forward kinematics was successful.
        bool getForwardKinematics(const Vector3 &joint_angles, Vector3 &foot_position) override;

        /// @brief Inverse kinematics for FiveBar2D leg.
        /// @param foot_position Foot position.
        /// @param joint_angles Joint angles.
        /// @return If the inverse kinematics was successful.
        bool getInverseKinematics(const Vector3 &foot_position, Vector3 &joint_angles) override;

        /// @brief Jacobian for FiveBar2D leg.
        /// @param joint_angles Joint angles.
        /// @param jacobian Jacobian matrix.
        /// @return If the Jacobian matrix was successful.
        bool getJacobian(const Vector3 &joint_angles, Matrix3 &jacobian) override;

        /// @brief Flip the leg in the Y axis.
        void flipYAxis();

    private:
        Float d_, lt_, lc_, a_axis_;
    };

}

#endif