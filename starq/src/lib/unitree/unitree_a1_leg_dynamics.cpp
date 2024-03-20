#include "starq/unitree/unitree_a1_leg_dynamics.hpp"

#include <cmath>

namespace starq::unitree
{

    UnitreeA1LegDynamics::UnitreeA1LegDynamics(Float d, Float lt, Float lc)
        : d_(d), lt_(lt), lc_(lc), a_axis_(1)
    {
    }

    bool UnitreeA1LegDynamics::getForwardKinematics(const Vector3 &joint_angles, Vector3 &foot_position)
    {
        const Float q1 = joint_angles(0);
        const Float q2 = joint_angles(1);
        const Float q3 = joint_angles(2);

        const Float s1 = std::sin(q1);
        const Float c1 = std::cos(q1);
        const Float s2 = std::sin(q2);
        const Float c2 = std::cos(q2);
        const Float s3 = std::sin(q3);
        const Float c3 = std::cos(q3);
        const Float s23 = std::sin(q2 + q3);

        const Float x = -lc_ * s23 - lt_ * s2;
        const Float y = a_axis_ * d_ * c1 - lc_ * (s1 * s2 * s3 - c2 * c3 * s1) + lt_ * c2 * q1;
        const Float z = a_axis_ * d_ * s1 - lc_ * (c1 * c2 * c3 - c1 * s2 * s3) - lt_ * c1 * c2;

        foot_position << x, y, z;

        return true;
    }

    bool UnitreeA1LegDynamics::getInverseKinematics(const Vector3 &foot_position, Vector3 &joint_angles)
    {

        const Float x = foot_position(0);
        const Float y = a_axis_ * foot_position(1);
        const Float z = foot_position(2);

        // Get q1 from Y-Z plane
        const Float r1 = std::sqrt(y * y + z * z);
        const Float alpha = std::asin(d_ / r1);
        const Float beta = M_PI_2 - alpha;
        const Float gamma = std::atan2(z, y);
        const Float q1 = beta + gamma;

        // Transform to the new coordinate system
        const Float dy = y - d_ * std::cos(q1);
        const Float dz = z - d_ * std::sin(q1);

        const Float zp = -dy * sin(q1) + dz * cos(q1);

        const Float r2 = std::sqrt(x * x + zp * zp);
        const Float phi = std::atan2(zp, x);
        const Float f = phi + M_PI_2;
        const Float lambda = std::acos((r2 * r2 + lt_ * lt_ - lc_ * lc_) / (2 * r2 * lt_));
        const Float del = std::asin(lt_ * std::sin(lambda) / lc_);
        const Float epsilon = M_PI - del - lambda;
        const Float q2 = f - lambda;
        const Float q3 = M_PI - epsilon;

        joint_angles << q1 * a_axis_, -q2, -q3;

        return true;
    }

    bool UnitreeA1LegDynamics::getJacobian(const Vector3 &joint_angles, Matrix3 &jacobian)
    {

        const Float q1 = joint_angles(0);
        const Float q2 = joint_angles(1);
        const Float q3 = joint_angles(2);

        const Float s1 = std::sin(q1);
        const Float c1 = std::cos(q1);
        const Float s2 = std::sin(q2);
        const Float c2 = std::cos(q2);
        const Float s3 = std::sin(q3);
        const Float c3 = std::cos(q3);
        const Float s23 = std::sin(q2 + q3);
        const Float c23 = std::cos(q2 + q3);

        const Float dXdq1 = 0.0;
        const Float dXdq2 = -lc_ * c23 - lt_ * c2;
        const Float dXdq3 = -lc_ * c23;
        const Float dYdq1 = lc_ * (c1 * c2 * c3 - c1 * s2 * s3) - a_axis_ *  d_ * s1 + lt_ * c1 * c2;
        const Float dYdq2 = -s1 * (lc_ * s23 + lt_ * s2);
        const Float dYdq3 = -lc_ * s1 * s23;
        const Float dZdq1 = a_axis_ * d_ * c1 - lc_ * (s1 * s2 * s3 - c2 * c3 * s1) + lt_ * c2 * s1;
        const Float dZdq2 = c1 * (lc_ * s23 + lt_ * s2);
        const Float dZdq3 = lc_ * c1 * s23;

        jacobian << dXdq1, dXdq2, dXdq3,
            dYdq1, dYdq2, dYdq3,
            dZdq1, dZdq2, dZdq3;

        return true;
    }

    void UnitreeA1LegDynamics::flipYAxis()
    {
        a_axis_ = -1;
    }

}