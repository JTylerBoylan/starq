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
        bool setFootPosition(const VectorXf &foot_position,
                             const VectorXf &foot_velocity_ff = VectorXf(),
                             const VectorXf &foot_accleration_ff = VectorXf()) override;

        /// @brief Set the inertia matrix for a leg.
        /// @param inertia_matrix Inertia matrix.
        void setInertiaMatrix(const MatrixXf &inertia_matrix);

        /// @brief Set the coriolis matrix for a leg.
        /// @param coriolis_matrix Coriolis matrix.
        void setCoriolisMatrix(const MatrixXf &coriolis_matrix);

        /// @brief Set the gravity vector for a leg.
        /// @param gravity_vector Gravity vector.
        void setGravityVector(const VectorXf &gravity_vector);

        /// @brief Set the kp matrix for a leg.
        /// @param kp_matrix Kp matrix.
        void setKpMatrix(const MatrixXf &kp_matrix);

        /// @brief Set the kd matrix for a leg.
        /// @param kd_matrix Kd matrix.
        void setKdMatrix(const MatrixXf &kd_matrix);

    private:
        MatrixXf inertia_matrix_;
        MatrixXf d_inertia_matrix_dt_;
        MatrixXf coriolis_matrix_;
        VectorXf gravity_vector_;
        MatrixXf kp_matrix_;
        MatrixXf kd_matrix_;
    };
}

#endif