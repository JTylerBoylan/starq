#include "starq/mpc/foothold_planner.hpp"

namespace starq::mpc
{

    FootholdPlanner::FootholdPlanner(std::vector<LegController::Ptr> legs,
                                     starq::RobotDynamics::Ptr robot_dynamics,
                                     starq::slam::Localization::Ptr localization)
        : legs_(legs),
          robot_dynamics_(robot_dynamics),
          localization_(localization)
    {
    }

    FootholdPlanner::~FootholdPlanner()
    {
    }

    bool FootholdPlanner::configure(const size_t N,
                                    const StanceTrajectory &stance_traj, const GaitSequence &gait_seq,
                                    const ReferenceTrajectory &ref_traj, FootholdTrajectory &foothold_traj) const
    {
        foothold_traj.resize(N);

        const Vector3f current_position = localization_->getCurrentPosition();
        const Vector3f current_orientation = localization_->getCurrentOrientation();
        const Matrix3f current_rotation = localization_->toRotationMatrix(current_orientation);

        const auto hip_locations = robot_dynamics_->getHipLocations();

        const size_t num_legs = legs_.size();
        foothold_traj[0].resize(num_legs);
        for (size_t j = 0; j < num_legs; j++)
        {
            VectorXf leg_position_hip;
            legs_[j]->getFootPositionEstimate(leg_position_hip);
            const Vector3f leg_position_body = hip_locations[j] + leg_position_hip;
            const Vector3f leg_position_world = current_position + current_rotation * leg_position_body;

            if (stance_traj[0][j])
                foothold_traj[0][j] = leg_position_world;
            else
                foothold_traj[0][j] = current_position;
        }

        for (size_t i = 1; i < N; i++)
        {
            const Vector3f position_i = ref_traj[i].position;
            const Vector3f orientation_i = ref_traj[i].orientation;
            const Matrix3f rotation_i = localization_->toRotationMatrix(orientation_i);

            const size_t num_legs_i = stance_traj[i].size();
            foothold_traj[i].resize(num_legs_i);
            for (size_t j = 0; j < num_legs_i; j++)
            {
                const bool curr_stance = stance_traj[i][j];
                const bool last_stance = stance_traj[i - 1][j];

                if (curr_stance && last_stance)
                {
                    foothold_traj[i][j] = foothold_traj[i - 1][j];
                }
                else if (curr_stance && !last_stance)
                {
                    const Vector3f hip_position_i = position_i + rotation_i * hip_locations[j];
                    const Vector3f body_velocity = rotation_i * ref_traj[i].linear_velocity;
                    const milliseconds stance_duration = gait_seq[i]->getStanceDuration();

                    Vector3f leg_position_world = hip_position_i + 0.5f * body_velocity * stance_duration.count() * 1E-3f;
                    leg_position_world.z() = 0.0;

                    foothold_traj[i][j] = leg_position_world;
                }
                else
                {
                    foothold_traj[i][j] = position_i;
                }
            }
        }
        return true;
    }

}