#include "starq/dynamics/unitree_rrr.hpp"

#include <cmath>

namespace starq::dynamics
{

    Unitree_RRR::Unitree_RRR(float d, float lt, float lc)
        : d_(d), lt_(lt), lc_(lc), a_axis_(1)
    {
    }

    bool Unitree_RRR::getForwardKinematics(const VectorXf &joint_angles, VectorXf &foot_position)
    {
        if (joint_angles.size() != 3)
            return false;

        const float q1 = joint_angles(0);
        const float q2 = joint_angles(1);
        const float q3 = joint_angles(2);

        const float s1 = sin(q1);
        const float c1 = cos(q1);
        const float s2 = sin(q2);
        const float c2 = cos(q2);
        const float s3 = sin(q3);
        const float c3 = cos(q3);
        const float s23 = sin(q2 + q3);

        const float x = -lc_ * s23 - lt_ * s2;
        const float y = a_axis_ * d_ * c1 - lc_ * (s1 * s2 * s3 - c2 * c3 * s1) + lt_ * c2 * q1;
        const float z = a_axis_ * d_ * s1 - lc_ * (c1 * c2 * c3 - c1 * s2 * s3) - lt_ * c1 * c2;

        foot_position.resize(3);
        foot_position << x, y, z;

        return true;
    }

    bool Unitree_RRR::getInverseKinematics(const VectorXf &foot_position, VectorXf &joint_angles)
    {

        if (foot_position.size() != 3)
            return false;

        const float x = foot_position(0);
        const float y = foot_position(1);
        const float z = foot_position(2);

        // Get q1 from Y-Z plane
        const float r1 = std::sqrt(y * y + z * z);
        const float alpha = std::asin(d_ / r1);
        const float beta = M_PI_2 - alpha;
        const float gamma = std::atan2(z, y);
        const float q1 = beta + gamma;

        // Transform to the new coordinate system
        const float dy = y - d_ * std::cos(q1);
        const float dz = z - d_ * std::sin(q1);

        const float zp = -dy * sin(q1) + dz * cos(q1);

        const float r2 = std::sqrt(x * x + zp * zp);
        const float phi = std::atan2(zp, x);
        const float f = phi + M_PI_2;
        const float lambda = std::acos((r2 * r2 + lt_ * lt_ - lc_ * lc_) / (2 * r2 * lt_));
        const float del = std::asin(lt_ * std::sin(lambda) / lc_);
        const float epsilon = M_PI - del - lambda;
        const float q2 = f - lambda;
        const float q3 = M_PI - epsilon;

        joint_angles.resize(3);
        joint_angles << q1 * a_axis_, -q2, -q3;

        return true;
    }

    bool Unitree_RRR::getJacobian(const VectorXf &joint_angles, MatrixXf &jacobian)
    {

        if (joint_angles.size() != 3)
            return false;

        const float q1 = joint_angles(0);
        const float q2 = joint_angles(1);
        const float q3 = joint_angles(2);

        const float s1 = sin(q1);
        const float c1 = cos(q1);
        const float s2 = sin(q2);
        const float c2 = cos(q2);
        const float s3 = sin(q3);
        const float c3 = cos(q3);
        const float s23 = sin(q2 + q3);
        const float c23 = cos(q2 + q3);

        const float dXdq1 = 0.0;
        const float dXdq2 = -lc_ * c23 - lt_ * c2;
        const float dXdq3 = -lc_ * c23;
        const float dYdq1 = lc_ * (c1 * c2 * c3 - c1 * s2 * s3) - a_axis_ *  d_ * s1 + lt_ * c1 * c2;
        const float dYdq2 = -s1 * (lc_ * s23 + lt_ * s2);
        const float dYdq3 = -lc_ * s1 * s23;
        const float dZdq1 = a_axis_ * d_ * c1 - lc_ * (s1 * s2 * s3 - c2 * c3 * s1) + lt_ * c2 * s1;
        const float dZdq2 = c1 * (lc_ * s23 + lt_ * s2);
        const float dZdq3 = lc_ * c1 * s23;

        jacobian.resize(3, 3);
        jacobian << dXdq1, dXdq2, dXdq3,
            dYdq1, dYdq2, dYdq3,
            dZdq1, dZdq2, dZdq3;

        return true;
    }

    void Unitree_RRR::flipYAxis()
    {
        a_axis_ = -1;
    }

}