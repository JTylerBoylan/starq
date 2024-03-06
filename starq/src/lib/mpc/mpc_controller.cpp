#include "starq/mpc/mpc_controller.hpp"

#include <iostream>

namespace starq::mpc
{

    MPCController::MPCController(const MPCConfiguration::Ptr config,
                                 const MPCSolver::Ptr solver,
                                 const LegCommandPublisher::Ptr leg_command_publisher)
        : config_(config),
          solver_(solver),
          leg_command_publisher_(leg_command_publisher),
          legs_(config->getLegControllers()),
          localization_(config->getLocalization()),
          robot_dynamics_(config->getRobotDynamics()),
          stop_on_fail_(false),
          sleep_duration_us_(1000),
          step_height_(0.075f),
          swing_resolution_(100)
    {
        last_force_state_.resize(legs_.size(),
                                 std::make_pair(true, Vector3f::Zero()));

        trajectory_publishers_.resize(legs_.size());
        for (size_t i = 0; i < legs_.size(); i++)
        {
            trajectory_publishers_[i] = std::make_shared<TrajectoryPublisher>(leg_command_publisher_);
        }
    }

    MPCController::~MPCController()
    {
        if (isRunning())
            stop();
    }

    void MPCController::run()
    {
        while (isRunning())
        {
            std::this_thread::sleep_for(sleep_duration_us_);

            if (!config_->isReady())
            {
                continue;
            }
            

            if (!solver_->update(config_))
            {
                if (stop_on_fail_)
                    break;
                else
                    continue;
            }

            if (!solver_->solve())
            {
                if (stop_on_fail_)
                    break;
                else
                    continue;
            }

            FootForceState force_state = solver_->getFirstForceState();

            for (size_t j = 0; j < force_state.size(); j++)
            {
                if (force_state[j].first)
                {
                    sendFootForce(j, -force_state[j].second);
                }
                else if (last_force_state_[j].first)
                {
                    sendSwingTrajectory(j);
                }
            }

            last_force_state_ = force_state;
        }

        stop();
    }

    void MPCController::sendFootForce(const uint8_t leg_id, const Vector3f &force_world)
    {
        const Vector3f orientation = localization_->getCurrentOrientation();
        const Matrix3f rotation = localization_->toRotationMatrix(orientation);
        const Vector3f force_body = rotation.transpose() * force_world;

        LegCommand::Ptr command = std::make_shared<LegCommand>();
        command->control_mode = ControlMode::TORQUE;
        command->target_force = force_body;
        command->leg_id = leg_id;

        leg_command_publisher_->sendCommand(command);
    }

    void MPCController::sendSwingTrajectory(const uint8_t leg_id)
    {
        const milliseconds swing_duration = config_->getGait(0)->getSwingDuration();
        const milliseconds stance_duration = config_->getGait(0)->getStanceDuration();
        const Vector3f velocity = config_->getReferenceState(1).linear_velocity;

        VectorXf start_position;
        leg_command_publisher_->getLegControllers()[leg_id]->getFootPositionEstimate(start_position);
        Vector3f end_position = 0.5f * velocity * stance_duration.count() * 1E-3f +
                                robot_dynamics_->getDefaultFootLocations()[leg_id];

        const Vector3f delta = end_position - start_position.head(3);
        const float radius = 0.5f * delta.norm();

        const float angle = std::atan2(delta.y(), delta.x());
        Matrix3f rotation;
        rotation = AngleAxisf(angle, Vector3f::UnitZ());

        std::vector<LegCommand::Ptr> trajectory;
        for (size_t i = 0; i <= swing_resolution_; i++)
        {
            const float ratio = static_cast<float>(i) / static_cast<float>(swing_resolution_);
            const float s = M_PI * ratio;
            const float px = radius * (1 - std::cos(s));
            const float pz = step_height_ * std::sin(s);
            const Vector3f arc_position = Vector3f(px, 0.0f, pz);
            const Vector3f position = rotation * arc_position + start_position.head(3);

            LegCommand::Ptr command = std::make_shared<LegCommand>();
            command->control_mode = ControlMode::POSITION;
            command->leg_id = leg_id;
            command->delay = milliseconds(time_t(swing_duration.count() * ratio));
            command->target_position = position;
            trajectory.push_back(command);
        }

        trajectory_publishers_[leg_id]->runTrajectory(trajectory);
    }

}