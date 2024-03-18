#ifndef STARQ__OPERATIONAL_SPACE_CONTROLLER_HPP_
#define STARQ__OPERATIONAL_SPACE_CONTROLLER_HPP_

#include "starq/leg_controller.hpp"

namespace starq
{

    /// @brief Uses LegDynamics to convert leg commands into motor commands
    class OperationalSpaceController : public LegController
    {
    public:
        using Ptr = std::shared_ptr<OperationalSpaceController>;

        /// @brief Create a leg controller.
        /// @param controller Motor controller.
        OperationalSpaceController(const starq::LegDynamics::Ptr dynamics,
                                   const std::vector<MotorController::Ptr> motor_controllers);

        /// @brief Destroy the leg controller.
        ~OperationalSpaceController();

        /// @brief Set the foot position for a leg.
        /// @param foot_position Foot position.
        /// @param foot_velocity_ff Foot velocity feedforward.
        /// @param foot_accleration_ff Foot acceleration feedforward.
        /// @return If the command was sent successfully.
        bool setFootPosition(const Vector3 &foot_position,
                             const Vector3 &foot_velocity_ff = Vector3::Zero(),
                             const Vector3 &foot_accleration_ff = Vector3::Zero()) override;

        /// @brief Set the inertia matrix for a leg.
        /// @param inertia_matrix Inertia matrix.
        void setInertiaMatrix(const Matrix3 &inertia_matrix);

        /// @brief Set the coriolis matrix for a leg.
        /// @param coriolis_matrix Coriolis matrix.
        void setCoriolisMatrix(const Matrix3 &coriolis_matrix);

        /// @brief Set the gravity vector for a leg.
        /// @param gravity_vector Gravity vector.
        void setGravityVector(const Vector3 &gravity_vector);

        /// @brief Set the kp matrix for a leg.
        /// @param kp_matrix Kp matrix.
        void setKpMatrix(const Matrix3 &kp_matrix);

        /// @brief Set the kd matrix for a leg.
        /// @param kd_matrix Kd matrix.
        void setKdMatrix(const Matrix3 &kd_matrix);

    private:
        Matrix3 inertia_matrix_;
        Matrix3 d_inertia_matrix_dt_;
        Matrix3 coriolis_matrix_;
        Vector3 gravity_vector_;
        Matrix3 kp_matrix_;
        Matrix3 kd_matrix_;
    };
}

#endif