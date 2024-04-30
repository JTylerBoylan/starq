#include "starq/mpc/foothold_planner.hpp"

namespace starq::mpc
{

    FootholdPlanner::FootholdPlanner(std::vector<LegController::Ptr> legs,
                                     starq::RobotParameters::Ptr robot_parameters,
                                     starq::slam::Localization::Ptr localization)
        : legs_(legs),
          robot_parameters_(robot_parameters),
          localization_(localization)
    {
    }

    FootholdPlanner::~FootholdPlanner()
    {
    }

    bool FootholdPlanner::configure(const size_t N, const milliseconds dt,
                                    const StanceTrajectory &stance_traj, const GaitSequence &gait_seq,
                                    const ReferenceTrajectory &ref_traj, FootholdTrajectory &foothold_traj) const
    {
        foothold_traj.resize(N);

        const auto hip_locations = robot_parameters_->getHipLocations();
        const auto def_foot_positions = robot_parameters_->getDefaultFootLocations();

        const size_t num_legs = legs_.size();
        foothold_traj[0].resize(num_legs);
        for (size_t j = 0; j < num_legs; j++)
        {
            if (stance_traj[0][j])
            {
                Vector3 leg_position_hip;
                legs_[j]->getFootPositionEstimate(leg_position_hip);
                const Vector3 leg_position_body = hip_locations[j] + leg_position_hip;
                foothold_traj[0][j] = leg_position_body;
            }
            else
            {
                foothold_traj[0][j] = Vector3::Zero();
            }
        }

        for (size_t i = 1; i < N; i++)
        {
            const size_t num_legs_i = stance_traj[i].size();
            foothold_traj[i].resize(num_legs_i);
            for (size_t j = 0; j < num_legs_i; j++)
            {
                const bool curr_stance = stance_traj[i][j];
                const bool last_stance = stance_traj[i - 1][j];

                if (curr_stance && last_stance)
                {
                    const Vector3 delta_foot_position = -ref_traj[i].linear_velocity * dt.count() * 1E-3f;
                    foothold_traj[i][j] = foothold_traj[i - 1][j] + delta_foot_position;
                }
                else if (curr_stance && !last_stance)
                {
                    const Vector3 def_foot_position_body = hip_locations[j] + def_foot_positions[j];
                    const Vector3 hip_velocity_i = ref_traj[i].linear_velocity +
                                                   ref_traj[i].angular_velocity.cross(hip_locations[j]);
                    const milliseconds stance_duration = gait_seq[i]->getStanceDuration();

                    const Vector3 leg_position = def_foot_position_body +
                                                 0.5f * hip_velocity_i * stance_duration.count() * 1E-3f;

                    foothold_traj[i][j] = leg_position;
                }
                else
                {
                    foothold_traj[i][j] = Vector3::Zero();
                }
            }
        }
        return true;
    }

}