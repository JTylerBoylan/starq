#include "starq/starq/starq_fivebar2d_leg_dynamics.hpp"

#include <cmath>

namespace starq
{

    STARQFiveBar2DLegDynamics::STARQFiveBar2DLegDynamics(Float L1, Float L2)
        : L1_(L1), L2_(L2), y_axis_(1.0) {}

    bool STARQFiveBar2DLegDynamics::getForwardKinematics(const Vector3 &joint_angles, Vector3 &foot_position)
    {
        const Float thetaA = y_axis_ * joint_angles(0);
        const Float thetaB = y_axis_ * joint_angles(1);

        const Float alpha = 0.5f * (M_PI - thetaA - thetaB);
        const Float gamma = std::asin(L1_ * std::sin(alpha) / L2_);
        const Float phi = M_PI - alpha - gamma;

        const Float theta = -(thetaA + alpha);
        const Float R = L2_ * std::sin(phi) / std::sin(alpha);

        const Float X = R * std::cos(theta);
        const Float Z = R * std::sin(theta);

        foot_position = Vector3(X, 0, Z);

        return true;
    }

    bool STARQFiveBar2DLegDynamics::getInverseKinematics(const Vector3 &foot_position, Vector3 &joint_angles)
    {

        const Float X = foot_position.x();
        const Float Z = foot_position.z();

        const Float theta0 = std::atan2(-Z, X);
        const Float theta1 = std::atan2(-Z, -X);
        const Float R = std::sqrt(X * X + Z * Z);
        const Float alpha = std::acos((R * R + L1_ * L1_ - L2_ * L2_) / (2.0f * R * L1_));

        const Float thetaA = theta0 - alpha;
        const Float thetaB = theta1 - alpha;

        joint_angles = Vector3(y_axis_ * thetaA, y_axis_ * thetaB, 0);

        return true;
    }

    bool STARQFiveBar2DLegDynamics::getJacobian(const Vector3 &joint_angles, Matrix3 &jacobian)
    {

        const Float thetaA = y_axis_ * joint_angles(0);
        const Float thetaB = y_axis_ * joint_angles(1);

        const Float thetaA_2 = 0.5 * thetaA;
        const Float thetaB_2 = 0.5 * thetaB;

        const Float dXdA = (L2_ * std::sin(thetaA_2 - thetaB_2) * std::sin(thetaA_2 + thetaB_2 - std::asin((L1_ * std::cos(thetaA_2 + thetaB_2)) / L2_)) * ((M_SQRT2 * L1_ * std::sin(thetaA_2 + thetaB_2)) / (2 * L2_ * std::sqrt(-(std::pow(L1_, 2) - 2 * std::pow(L2_, 2) + std::pow(L1_, 2) * std::cos(thetaA + thetaB)) / std::pow(L2_, 2))) + 0.5f)) / std::cos(thetaA_2 + thetaB_2) - (L2_ * std::sin(thetaA_2 - thetaB_2) * std::sin(thetaA_2 + thetaB_2) * std::cos(thetaA_2 + thetaB_2 - std::asin((L1_ * std::cos(thetaA_2 + thetaB_2)) / L2_))) / (2 * std::pow(std::cos(thetaA_2 + thetaB_2), 2)) - (L2_ * std::cos(thetaA_2 - thetaB_2) * std::cos(thetaA_2 + thetaB_2 - std::asin((L1_ * std::cos(thetaA_2 + thetaB_2)) / L2_))) / (2 * std::cos(thetaA_2 + thetaB_2));
        const Float dXdB = (L2_ * std::cos(thetaA_2 - thetaB_2) * std::cos(thetaA_2 + thetaB_2 - std::asin((L1_ * std::cos(thetaA_2 + thetaB_2)) / L2_))) / (2 * std::cos(thetaA_2 + thetaB_2)) - (L2_ * std::sin(thetaA_2 - thetaB_2) * std::sin(thetaA_2 + thetaB_2) * std::cos(thetaA_2 + thetaB_2 - std::asin((L1_ * std::cos(thetaA_2 + thetaB_2)) / L2_))) / (2 * std::pow(std::cos(thetaA_2 + thetaB_2), 2)) + (L2_ * std::sin(thetaA_2 - thetaB_2) * std::sin(thetaA_2 + thetaB_2 - std::asin((L1_ * std::cos(thetaA_2 + thetaB_2)) / L2_)) * ((M_SQRT2 * L1_ * std::sin(thetaA_2 + thetaB_2)) / (2 * L2_ * std::sqrt(-(std::pow(L1_, 2) - 2 * std::pow(L2_, 2) + std::pow(L1_, 2) * std::cos(thetaA + thetaB)) / std::pow(L2_, 2))) + 0.5f)) / std::cos(thetaA_2 + thetaB_2);
        const Float dXdC = 0.0;
        const Float dYdA = 0.0;
        const Float dYdB = 0.0;
        const Float dYdC = 0.0;
        const Float dZdA = (L2_ * std::sin(thetaA_2 - thetaB_2) * std::cos(thetaA_2 + thetaB_2 - std::asin((L1_ * std::cos(thetaA_2 + thetaB_2)) / L2_))) / (2 * std::cos(thetaA_2 + thetaB_2)) - (L2_ * std::cos(thetaA_2 - thetaB_2) * std::sin(thetaA_2 + thetaB_2) * std::cos(thetaA_2 + thetaB_2 - std::asin((L1_ * std::cos(thetaA_2 + thetaB_2)) / L2_))) / (2 * std::pow(std::cos(thetaA_2 + thetaB_2), 2)) + (L2_ * std::cos(thetaA_2 - thetaB_2) * std::sin(thetaA_2 + thetaB_2 - std::asin((L1_ * std::cos(thetaA_2 + thetaB_2)) / L2_)) * ((M_SQRT2 * L1_ * std::sin(thetaA_2 + thetaB_2)) / (2 * L2_ * std::sqrt(-(std::pow(L1_, 2) - 2 * std::pow(L2_, 2) + std::pow(L1_, 2) * std::cos(thetaA + thetaB)) / std::pow(L2_, 2))) + 0.5f)) / std::cos(thetaA_2 + thetaB_2);
        const Float dZdB = (L2_ * std::cos(thetaA_2 - thetaB_2) * std::sin(thetaA_2 + thetaB_2 - std::asin((L1_ * std::cos(thetaA_2 + thetaB_2)) / L2_)) * ((M_SQRT2 * L1_ * std::sin(thetaA_2 + thetaB_2)) / (2 * L2_ * std::sqrt(-(std::pow(L1_, 2) - 2 * std::pow(L2_, 2) + std::pow(L1_, 2) * std::cos(thetaA + thetaB)) / std::pow(L2_, 2))) + 0.5f)) / std::cos(thetaA_2 + thetaB_2) - (L2_ * std::cos(thetaA_2 - thetaB_2) * std::sin(thetaA_2 + thetaB_2) * std::cos(thetaA_2 + thetaB_2 - std::asin((L1_ * std::cos(thetaA_2 + thetaB_2)) / L2_))) / (2 * std::pow(std::cos(thetaA_2 + thetaB_2), 2)) - (L2_ * std::sin(thetaA_2 - thetaB_2) * std::cos(thetaA_2 + thetaB_2 - std::asin((L1_ * std::cos(thetaA_2 + thetaB_2)) / L2_))) / (2 * std::cos(thetaA_2 + thetaB_2));
        const Float dZdC = 0.0;

        jacobian << dXdA, dXdB, dXdC,
            dYdA, dYdB, dYdC,
            dZdA, dZdB, dZdC;

        return true;
    }

    void STARQFiveBar2DLegDynamics::flipY()
    {
        y_axis_ = -y_axis_;
    }

}