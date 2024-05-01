#ifndef STARQ_DYNAMICS__STARQ_FIVEBAR2D_LEG_DYNAMICS_HPP_
#define STARQ_DYNAMICS__STARQ_FIVEBAR2D_LEG_DYNAMICS_HPP_

#include "starq/leg_dynamics.hpp"

namespace starq
{

    /// @brief Implementation of LegDynamics for the 2D symmetric five-bar leg
    class STARQFiveBar2DLegDynamics : public LegDynamics
    {
    public:
        using Ptr = std::shared_ptr<STARQFiveBar2DLegDynamics>;

        /// @brief Constructor for FiveBar2D leg.
        /// @param L1 Length of the first link in meters
        /// @param L2 Length of the second link in meters
        STARQFiveBar2DLegDynamics(Float L1, Float L2);

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

        /// @brief Flip the y axis of the leg.
        void flipY();

    private:
        Float L1_, L2_;
        Float y_axis_;
    };

}

#endif