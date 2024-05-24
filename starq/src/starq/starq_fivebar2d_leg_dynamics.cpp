#include "starq/starq/starq_fivebar2d_leg_dynamics.hpp"

#include <cmath>

namespace starq
{

    STARQFiveBar2DLegDynamics::STARQFiveBar2DLegDynamics(Float L1, Float L2)
        : L1_(L1), L2_(L2), YA_(1.0) {}

    bool STARQFiveBar2DLegDynamics::getForwardKinematics(const Vector3 &joint_angles, Vector3 &foot_position)
    {
        const Float thetaA = YA_ * joint_angles(0);
        const Float thetaB = YA_ * joint_angles(1);

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

        joint_angles = Vector3(YA_ * thetaA, YA_ * thetaB, 0);

        return true;
    }

    bool STARQFiveBar2DLegDynamics::getJacobian(const Vector3 &joint_angles, Matrix3 &jacobian)
    {
        using namespace std;

        const Float A = joint_angles(0);
        const Float B = joint_angles(1);

        const Float Y_2 = YA_ / 2.0;
        const Float Y_2A = Y_2 * A;
        const Float Y_2B = Y_2 * B;
        const Float s1 = sin(Y_2A + Y_2B);
        const Float c1 = cos(Y_2A + Y_2B);
        const Float s2 = sin(Y_2A - Y_2B);
        const Float c2 = cos(Y_2A - Y_2B);

        const Float as1 = Y_2A + Y_2B - asin((L1_ * c1) / L2_);
        
        const Float t1 = sin(as1) * (Y_2 + (L1_ * Y_2 * s1) / (L2_ * sqrt(-(L1_ * L1_ * c1 * c1 - L2_ * L2_) / (L2_ * L2_))));

        const Float t2 = L2_ * t1 / c1;
        const Float t3 = L2_ * Y_2 * cos(as1) / c1;
        const Float t4 = t3 * s1 / c1;

        const Float dXdA = t2 * s2 - t3 * c2 - t4 * s2;
        const Float dXdB = t2 * s2 + t3 * c2 - t4 * s2;
        const Float dXdC = 0.0;
        const Float dYdA = 0.0;
        const Float dYdB = 0.0;
        const Float dYdC = 0.0;
        const Float dZdA = t2 * c2 + t3 * s2 - t4 * c2;
        const Float dZdB = t2 * c2 - t3 * s2 - t4 * c2;
        const Float dZdC = 0.0;

        jacobian << dXdA, dXdB, dXdC,
            dYdA, dYdB, dYdC,
            dZdA, dZdB, dZdC;

        return true;
    }

    void STARQFiveBar2DLegDynamics::flipY()
    {
        YA_ = -YA_;
    }

}