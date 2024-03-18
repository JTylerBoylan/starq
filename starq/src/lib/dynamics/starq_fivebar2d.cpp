#include "starq/dynamics/starq_fivebar2d.hpp"

#include <cmath>

namespace starq::dynamics
{

    STARQ_FiveBar2D::STARQ_FiveBar2D(Float L1, Float L2)
        : L1_(L1), L2_(L2) {}

    bool STARQ_FiveBar2D::getForwardKinematics(const Vector3 &joint_angles, Vector3 &foot_position)
    {
        const Float thetaA = joint_angles(0);
        const Float thetaB = joint_angles(1);

        const Float alpha = 0.5f * (M_PI - thetaA - thetaB);
        const Float gamma = std::asin(L1_ * std::sin(alpha) / L2_);
        const Float phi = M_PI - alpha - gamma;

        const Float theta = -(thetaA + alpha);
        const Float R = L2_ * std::sin(phi) / std::sin(alpha);

        const Float X = R * std::cos(theta);
        const Float Y = R * std::sin(theta);

        foot_position = Vector3(X, Y, 0);

        return true;
    }

    bool STARQ_FiveBar2D::getInverseKinematics(const Vector3 &foot_position, Vector3 &joint_angles)
    {

        const Float X = foot_position(0);
        const Float Y = foot_position(1);

        const Float theta0 = std::atan2(-Y, X);
        const Float theta1 = std::atan2(-Y, -X);
        const Float R = std::sqrt(X * X + Y * Y);
        const Float alpha = std::acos((R * R + L1_ * L1_ - L2_ * L2_) / (2.0f * R * L1_));

        const Float thetaA = theta0 - alpha;
        const Float thetaB = theta1 - alpha;

        joint_angles = Vector3(thetaA, thetaB, 0);

        return true;
    }

    bool STARQ_FiveBar2D::getJacobian(const Vector3 &joint_angles, Matrix3 &jacobian)
    {

        const Float thetaA = joint_angles(0);
        const Float thetaB = joint_angles(1);

        const Float thetaA_2 = 0.5 * thetaA;
        const Float thetaB_2 = 0.5 * thetaB;

        const Float dXdA = (L2_ * std::sin(thetaA_2 - thetaB_2) * std::sin(thetaA_2 + thetaB_2 - std::asin((L1_ * std::cos(thetaA_2 + thetaB_2)) / L2_)) * ((M_SQRT2 * L1_ * std::sin(thetaA_2 + thetaB_2)) / (2 * L2_ * std::sqrt(-(std::pow(L1_, 2) - 2 * std::pow(L2_, 2) + std::pow(L1_, 2) * std::cos(thetaA + thetaB)) / std::pow(L2_, 2))) + 0.5f)) / std::cos(thetaA_2 + thetaB_2) - (L2_ * std::sin(thetaA_2 - thetaB_2) * std::sin(thetaA_2 + thetaB_2) * std::cos(thetaA_2 + thetaB_2 - std::asin((L1_ * std::cos(thetaA_2 + thetaB_2)) / L2_))) / (2 * std::pow(std::cos(thetaA_2 + thetaB_2), 2)) - (L2_ * std::cos(thetaA_2 - thetaB_2) * std::cos(thetaA_2 + thetaB_2 - std::asin((L1_ * std::cos(thetaA_2 + thetaB_2)) / L2_))) / (2 * std::cos(thetaA_2 + thetaB_2));
        const Float dXdB = (L2_ * std::cos(thetaA_2 - thetaB_2) * std::cos(thetaA_2 + thetaB_2 - std::asin((L1_ * std::cos(thetaA_2 + thetaB_2)) / L2_))) / (2 * std::cos(thetaA_2 + thetaB_2)) - (L2_ * std::sin(thetaA_2 - thetaB_2) * std::sin(thetaA_2 + thetaB_2) * std::cos(thetaA_2 + thetaB_2 - std::asin((L1_ * std::cos(thetaA_2 + thetaB_2)) / L2_))) / (2 * std::pow(std::cos(thetaA_2 + thetaB_2), 2)) + (L2_ * std::sin(thetaA_2 - thetaB_2) * std::sin(thetaA_2 + thetaB_2 - std::asin((L1_ * std::cos(thetaA_2 + thetaB_2)) / L2_)) * ((M_SQRT2 * L1_ * std::sin(thetaA_2 + thetaB_2)) / (2 * L2_ * std::sqrt(-(std::pow(L1_, 2) - 2 * std::pow(L2_, 2) + std::pow(L1_, 2) * std::cos(thetaA + thetaB)) / std::pow(L2_, 2))) + 0.5f)) / std::cos(thetaA_2 + thetaB_2);
        const Float dXdC = 0.0;
        const Float dYdA = (L2_ * std::sin(thetaA_2 - thetaB_2) * std::cos(thetaA_2 + thetaB_2 - std::asin((L1_ * std::cos(thetaA_2 + thetaB_2)) / L2_))) / (2 * std::cos(thetaA_2 + thetaB_2)) - (L2_ * std::cos(thetaA_2 - thetaB_2) * std::sin(thetaA_2 + thetaB_2) * std::cos(thetaA_2 + thetaB_2 - std::asin((L1_ * std::cos(thetaA_2 + thetaB_2)) / L2_))) / (2 * std::pow(std::cos(thetaA_2 + thetaB_2), 2)) + (L2_ * std::cos(thetaA_2 - thetaB_2) * std::sin(thetaA_2 + thetaB_2 - std::asin((L1_ * std::cos(thetaA_2 + thetaB_2)) / L2_)) * ((M_SQRT2 * L1_ * std::sin(thetaA_2 + thetaB_2)) / (2 * L2_ * std::sqrt(-(std::pow(L1_, 2) - 2 * std::pow(L2_, 2) + std::pow(L1_, 2) * std::cos(thetaA + thetaB)) / std::pow(L2_, 2))) + 0.5f)) / std::cos(thetaA_2 + thetaB_2);
        const Float dYdB = (L2_ * std::cos(thetaA_2 - thetaB_2) * std::sin(thetaA_2 + thetaB_2 - std::asin((L1_ * std::cos(thetaA_2 + thetaB_2)) / L2_)) * ((M_SQRT2 * L1_ * std::sin(thetaA_2 + thetaB_2)) / (2 * L2_ * std::sqrt(-(std::pow(L1_, 2) - 2 * std::pow(L2_, 2) + std::pow(L1_, 2) * std::cos(thetaA + thetaB)) / std::pow(L2_, 2))) + 0.5f)) / std::cos(thetaA_2 + thetaB_2) - (L2_ * std::cos(thetaA_2 - thetaB_2) * std::sin(thetaA_2 + thetaB_2) * std::cos(thetaA_2 + thetaB_2 - std::asin((L1_ * std::cos(thetaA_2 + thetaB_2)) / L2_))) / (2 * std::pow(std::cos(thetaA_2 + thetaB_2), 2)) - (L2_ * std::sin(thetaA_2 - thetaB_2) * std::cos(thetaA_2 + thetaB_2 - std::asin((L1_ * std::cos(thetaA_2 + thetaB_2)) / L2_))) / (2 * std::cos(thetaA_2 + thetaB_2));
        const Float dYdC = 0.0;
        const Float dZdA = 0.0;
        const Float dZdB = 0.0;
        const Float dZdC = 1.0;

        jacobian << dXdA, dXdB, dXdC,
            dYdA, dYdB, dYdC,
            dZdA, dZdB, dZdC;

        return true;
    }

}