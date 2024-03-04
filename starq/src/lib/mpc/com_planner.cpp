#include "starq/mpc/com_planner.hpp"

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

        const float dT = dt.count() / 1000.0;
        for (size_t i = 1; i < N; i++)
        {
            const Vector3f linear_velocity = gait_seq[i - 1]->getLinearVelocity();
            const Vector3f angular_velocity = gait_seq[i - 1]->getAngularVelocity();

            const float last_theta = ref_traj[i - 1].orientation.z();
            const float last_x = ref_traj[i - 1].position.x();
            const float last_y = ref_traj[i - 1].position.y();

            float &theta = ref_traj[i].orientation.z();
            float &x = ref_traj[i].position.x();
            float &y = ref_traj[i].position.y();
            float &z = ref_traj[i].position.z();
            theta = last_theta + angular_velocity.z() * dT;
            x = last_x + linear_velocity.x() * std::cos(theta) * dT;
            y = last_y + linear_velocity.y() * std::sin(theta) * dT;
            z = robot_dynamics_->getBodyHeight();
        }
        return true;
    }

}