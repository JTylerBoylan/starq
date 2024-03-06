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
        ref_traj[0].angular_velocity = localization_->getCurrentAngularVelocity();

        // std::cout << "ref_traj[" << 0 << "].position: " << ref_traj[0].position.transpose() << std::endl;
        // std::cout << "ref_traj[" << 0 << "].orientation: " << ref_traj[0].orientation.transpose() << std::endl;
        // std::cout << "ref_traj[" << 0 << "].linear_velocity: " << ref_traj[0].linear_velocity.transpose() << std::endl;
        // std::cout << "ref_traj[" << 0 << "].angular_velocity: " << ref_traj[0].angular_velocity.transpose() << std::endl;

        const float dT = dt.count() / 1000.0;
        for (size_t i = 1; i < N; i++)
        {

            Vector3f linear_velocity;
            Vector3f angular_velocity;
            switch (gait_seq[i - 1]->getControlMode())
            {
            case GAIT_POSITION_CONTROL:
            {
                const Vector3f delta_p = gait_seq[i]->getPosition() - ref_traj[i - 1].position;
                const Vector3f max_v = gait_seq[i - 1]->getMaxLinearVelocity();
                const Vector3f max_delta_p = max_v * dT;
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
                const Vector3f delta_o = gait_seq[i]->getOrientation() - ref_traj[i - 1].orientation;
                const Vector3f max_w = gait_seq[i - 1]->getMaxAngularVelocity();
                const Vector3f max_delta_o = max_w * dT;
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
                Matrix3f rotation;
                rotation = AngleAxisf(ref_traj[i - 1].orientation.z(), Vector3f::UnitZ());
                linear_velocity = rotation * gait_seq[i - 1]->getLinearVelocity();
                angular_velocity = rotation * gait_seq[i - 1]->getAngularVelocity();

                const float delta_z = robot_dynamics_->getStandingHeight() - ref_traj[i - 1].position.z();
                const float max_vz = gait_seq[i - 1]->getMaxLinearVelocity().z();
                const float max_delta_z = max_vz * dT;
                if (std::abs(delta_z) > max_delta_z)
                {
                    linear_velocity.z() = delta_z > 0 ? max_vz : -max_vz;
                }
                else
                {
                    linear_velocity.z() = delta_z / dT;
                }

                const Vector2f delta_o = -ref_traj[i - 1].orientation.head(2);
                const Vector2f max_w = gait_seq[i - 1]->getMaxAngularVelocity().head(2);
                const Vector2f max_delta_o = max_w * dT;
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
            ref_traj[i].angular_velocity = angular_velocity;

            // std::cout << "ref_traj[" << i << "].position: " << ref_traj[i].position.transpose() << std::endl;
            // std::cout << "ref_traj[" << i << "].orientation: " << ref_traj[i].orientation.transpose() << std::endl;
            // std::cout << "ref_traj[" << i << "].linear_velocity: " << ref_traj[i].linear_velocity.transpose() << std::endl;
            // std::cout << "ref_traj[" << i << "].angular_velocity: " << ref_traj[i].angular_velocity.transpose() << std::endl;
        }
        return true;
    }

}