#include "starq/mpc/foothold_planner.hpp"

namespace starq::mpc
{

    FootholdPlanner::FootholdPlanner(std::vector<LegController::Ptr> legs,
                                     std::vector<Vector3f> hip_locations,
                                     starq::slam::Localization::Ptr localization)
        : legs_(legs),
          hip_locations_(hip_locations),
          localization_(localization)
    {
    }

    FootholdPlanner::~FootholdPlanner()
    {
    }

    bool FootholdPlanner::configure(MPCConfiguration &config) const
    {
        const Vector3f current_com_position = localization_->getCurrentPosition();
        const Vector3f current_com_orientation = localization_->getCurrentOrientation();
        Matrix3f current_com_rotation;
        current_com_rotation = AngleAxisf(current_com_orientation.x(), Vector3f::UnitX()) *
                               AngleAxisf(current_com_orientation.y(), Vector3f::UnitY()) *
                               AngleAxisf(current_com_orientation.z(), Vector3f::UnitZ());

        config.foothold_trajectory[0].resize(legs_.size());
        for (size_t j = 0; j < legs_.size(); j++)
        {
            VectorXf leg_position_hip;
            legs_[j]->getFootPositionEstimate(leg_position_hip);
            const Vector3f leg_position_body = hip_locations_[j] + leg_position_hip;
            const Vector3f leg_position_world = current_com_position + current_com_rotation * leg_position_body;

            if (config.stance_trajectory[0][j])
                config.foothold_trajectory[0][j] = leg_position_world;
            else
                config.foothold_trajectory[0][j] = current_com_position;
        }

        for (size_t i = 1; i < config.window_size; i++)
        {
            const Vector3f com_position_i = config.com_trajectory[i].position;
            const Vector3f com_orientation_i = config.com_trajectory[i].orientation;
            Matrix3f com_rotation_i;
            com_rotation_i = AngleAxisf(com_orientation_i.x(), Vector3f::UnitX()) *
                             AngleAxisf(com_orientation_i.y(), Vector3f::UnitY()) *
                             AngleAxisf(com_orientation_i.z(), Vector3f::UnitZ());

            config.foothold_trajectory[i].resize(legs_.size());
            for (size_t j = 0; j < legs_.size(); j++)
            {
                const bool curr_stance = config.stance_trajectory[i][j];
                const bool last_stance = config.stance_trajectory[i - 1][j];

                if (curr_stance && last_stance)
                {
                    config.foothold_trajectory[i][j] = config.foothold_trajectory[i - 1][j];
                }
                else if (curr_stance && !last_stance)
                {
                    const Vector3f hip_position_i = com_position_i + com_rotation_i * hip_locations_[j];
                    const Vector3f body_velocity = com_rotation_i * config.com_trajectory[i].linear_velocity;
                    const milliseconds stance_duration = config.timing_trajectory[i].stance_duration;

                    Vector3f leg_position_world = hip_position_i + 0.5f * body_velocity * stance_duration.count() * 1E-3f;
                    leg_position_world.z() = com_position_i.z() - config.height;

                    config.foothold_trajectory[i][j] = leg_position_world;
                }
                else
                {
                    config.foothold_trajectory[i][j] = com_position_i;
                }
            }
        }
        return true;
    }

}