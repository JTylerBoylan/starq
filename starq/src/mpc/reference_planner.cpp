#include "starq/mpc/reference_planner.hpp"

#include <iostream>

namespace starq::mpc
{

    ReferencePlanner::ReferencePlanner(starq::slam::Localization::Ptr localization,
                                             starq::RobotParameters::Ptr robot_parameters)
        : localization_(localization),
          robot_parameters_(robot_parameters)
    {
    }

    ReferencePlanner::~ReferencePlanner()
    {
    }

    bool ReferencePlanner::configure(const size_t N, const milliseconds dt,
                                        const GaitSequence &gait_seq, ReferenceTrajectory &ref_traj) const
    {
        ref_traj.resize(N);

        ref_traj[0].position = localization_->getCurrentPosition();
        ref_traj[0].orientation = localization_->getCurrentOrientation();

        Matrix3 R = localization_->toRotationMatrix(ref_traj[0].orientation);
        ref_traj[0].linear_velocity = R.transpose() * localization_->getCurrentLinearVelocity();
        ref_traj[0].angular_velocity = localization_->getCurrentAngularVelocity();

        const Float dT = dt.count() / 1000.0;
        for (size_t i = 1; i < N; i++)
        {
            Vector3 linear_velocity;
            Vector3 angular_velocity;

            R = localization_->toRotationMatrix(ref_traj[i - 1].orientation);

            switch (gait_seq[i]->getControlMode())
            {
            case GAIT_POSITION_CONTROL:
            {
                const Vector3 delta_p = R.transpose() * (gait_seq[i]->getPosition() - ref_traj[i - 1].position);
                const Vector3 max_v = gait_seq[i]->getMaxLinearVelocity();
                const Vector3 v = delta_p / dT;
                linear_velocity = v.cwiseMin(max_v).cwiseMax(-max_v);

                Vector3 delta_o = gait_seq[i]->getOrientation() - ref_traj[i - 1].orientation;
                for (int j = 0; j < 3; j++)
                {
                    delta_o[j] = delta_o[j] - 2 * M_PI * floor((delta_o[j] + M_PI) / (2 * M_PI));
                }
                const Vector3 max_w = gait_seq[i]->getMaxAngularVelocity();
                const Vector3 w = delta_o / dT;
                angular_velocity = w.cwiseMin(max_w).cwiseMax(-max_w);
                break;
            }
            case GAIT_VELOCITY_CONTROL:
            {
                linear_velocity = gait_seq[i]->getLinearVelocity();
                angular_velocity = gait_seq[i]->getAngularVelocity();

                const Vector3 max_linear_velocity = gait_seq[i]->getMaxLinearVelocity();
                const Vector3 max_angular_velocity = gait_seq[i]->getMaxAngularVelocity();

                linear_velocity = linear_velocity.cwiseMin(max_linear_velocity).cwiseMax(-max_linear_velocity);
                angular_velocity = angular_velocity.cwiseMin(max_angular_velocity).cwiseMax(-max_angular_velocity);

                const Float delta_z = robot_parameters_->getStandingHeight() - ref_traj[i - 1].position.z();
                const Float max_vz = max_linear_velocity.z();
                const Float vz = delta_z / dT;
                linear_velocity.z() = std::max(std::min(vz, max_vz), -max_vz);

                const Eigen::Vector2<Float> delta_o = -ref_traj[i - 1].orientation.head(2);
                const Eigen::Vector2<Float> max_w = max_angular_velocity.head(2);
                const Eigen::Vector2<Float> vo = delta_o / dT;
                angular_velocity.head(2) = vo.cwiseMin(max_w).cwiseMax(-max_w);
                break;
            }
            }

            ref_traj[i].position = ref_traj[i - 1].position + R * linear_velocity * dT;
            ref_traj[i].orientation = ref_traj[i - 1].orientation + angular_velocity * dT;
            ref_traj[i].linear_velocity = linear_velocity;
            ref_traj[i].angular_velocity = angular_velocity;
        }
        return true;
    }

}