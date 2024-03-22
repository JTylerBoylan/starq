#include "starq/mpc/com_planner.hpp"

#include <iostream>

namespace starq::mpc
{

    CenterOfMassPlanner::CenterOfMassPlanner(starq::slam::Localization::Ptr localization,
                                             starq::RobotDynamics::Ptr robot_dynamics)
        : localization_(localization),
          robot_dynamics_(robot_dynamics)
    {
    }

    CenterOfMassPlanner::~CenterOfMassPlanner()
    {
    }

    bool CenterOfMassPlanner::configure(const size_t N, const milliseconds dt,
                                        const GaitSequence &gait_seq, ReferenceTrajectory &ref_traj) const
    {
        ref_traj.resize(N);

        ref_traj[0].position = localization_->getCurrentPosition();
        ref_traj[0].orientation = localization_->getCurrentOrientation();
        ref_traj[0].linear_velocity = localization_->getCurrentLinearVelocity();
        ref_traj[0].angular_velocity = getWorldAngularVelocity(ref_traj[0].orientation,
                                                               localization_->getCurrentAngularVelocity());

        const Float dT = dt.count() / 1000.0;
        for (size_t i = 1; i < N; i++)
        {

            Vector3 linear_velocity;
            Vector3 angular_velocity;
            switch (gait_seq[i]->getControlMode())
            {
            case GAIT_POSITION_CONTROL:
            {
                const Vector3 delta_p = gait_seq[i]->getPosition() - ref_traj[i - 1].position;
                const Vector3 max_v = gait_seq[i]->getMaxLinearVelocity();
                const Vector3 max_delta_p = max_v * dT;
                for (int j = 0; j < 3; j++)
                {
                    if (std::abs(delta_p[j]) > max_delta_p[j])
                    {
                        linear_velocity[j] = delta_p[j] > 0 ? max_v[j] : -max_v[j];
                    }
                    else
                    {
                        linear_velocity[j] = delta_p[j] / dT;
                    }
                }

                Vector3 delta_o = gait_seq[i]->getOrientation() - ref_traj[i - 1].orientation;
                for (int j = 0; j < 3; j++)
                {
                    if (delta_o[j] > M_PI)
                    {
                        delta_o[j] -= 2 * M_PI;
                    }
                    else if (delta_o[j] < -M_PI)
                    {
                        delta_o[j] += 2 * M_PI;
                    }
                }
                const Vector3 max_w = gait_seq[i]->getMaxAngularVelocity();
                const Vector3 max_delta_o = max_w * dT;
                for (int j = 0; j < 3; j++)
                {
                    if (std::abs(delta_o[j]) > max_delta_o[j])
                    {
                        angular_velocity[j] = delta_o[j] > 0 ? max_w[j] : -max_w[j];
                    }
                    else
                    {
                        angular_velocity[j] = delta_o[j] / dT;
                    }
                }
                break;
            }
            case GAIT_VELOCITY_CONTROL:
            {
                linear_velocity = gait_seq[i]->getLinearVelocity();
                angular_velocity = gait_seq[i]->getAngularVelocity();

                const Float delta_z = robot_dynamics_->getStandingHeight() - ref_traj[i - 1].position.z();
                const Float max_vz = gait_seq[i]->getMaxLinearVelocity().z();
                const Float max_delta_z = max_vz * dT;
                if (std::abs(delta_z) > max_delta_z)
                {
                    linear_velocity.z() = delta_z > 0 ? max_vz : -max_vz;
                }
                else
                {
                    linear_velocity.z() = delta_z / dT;
                }

                const Eigen::Vector2<Float> delta_o = -ref_traj[i - 1].orientation.head(2);
                const Eigen::Vector2<Float> max_w = gait_seq[i]->getMaxAngularVelocity().head(2);
                const Eigen::Vector2<Float> max_delta_o = max_w * dT;
                for (int j = 0; j < 2; j++)
                {
                    if (std::abs(delta_o[j]) > max_delta_o[j])
                    {
                        angular_velocity[j] = delta_o[j] > 0 ? max_w[j] : -max_w[j];
                    }
                    else
                    {
                        angular_velocity[j] = delta_o[j] / dT;
                    }
                }
                break;
            }
            }

            ref_traj[i].position = ref_traj[i - 1].position + linear_velocity * dT;
            ref_traj[i].orientation = ref_traj[i - 1].orientation + angular_velocity * dT;
            ref_traj[i].linear_velocity = linear_velocity;
            ref_traj[i].angular_velocity = getWorldAngularVelocity(ref_traj[i].orientation, angular_velocity);
        }
        return true;
    }

    Vector3 CenterOfMassPlanner::getWorldAngularVelocity(const Vector3 &orientation, const Vector3 &angular_velocity) const
    {
        Matrix3 R;
        R << cos(orientation.z()), -sin(orientation.z()), 0,
            sin(orientation.z()), cos(orientation.z()), 0,
            0, 0, 1;
        return R * angular_velocity;
    }

}