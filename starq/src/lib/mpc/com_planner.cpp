#include "starq/mpc/com_planner.hpp"

namespace starq::mpc
{

    CenterOfMassPlanner::CenterOfMassPlanner()
    {
    }

    CenterOfMassPlanner::~CenterOfMassPlanner()
    {
    }

    bool CenterOfMassPlanner::configure(MPCConfiguration &config) const
    {
        const float dt = config.time_step.count() / 1000.0;
        for (size_t i = 1; i < config.window_size; i++)
        {
            const Vector3f linear_velocity = config.com_trajectory[i - 1].linear_velocity;
            const Vector3f angular_velocity = config.com_trajectory[i - 1].angular_velocity;

            const float last_theta = config.com_trajectory[i - 1].orientation.z();
            const float last_x = config.com_trajectory[i - 1].position.x();
            const float last_y = config.com_trajectory[i - 1].position.y();

            float &theta = config.com_trajectory[i].orientation.z();
            float &x = config.com_trajectory[i].position.x();
            float &y = config.com_trajectory[i].position.y();
            theta = last_theta + angular_velocity.z() * dt;
            x = last_x + linear_velocity.x() * std::cos(theta) * dt;
            y = last_y + linear_velocity.y() * std::sin(theta) * dt;
        }
        return true;
    }

}