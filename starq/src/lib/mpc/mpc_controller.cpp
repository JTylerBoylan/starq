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
          swing_resolution_(100),
          swing_duration_factor_(0.5f)
    {
        last_force_state_.resize(legs_.size(),
                                 std::make_pair(true, Vector3::Zero()));

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

    void MPCController::sendFootForce(const uint8_t leg_id, const Vector3 &force_world)
    {
        const Vector3 orientation = localization_->getCurrentOrientation();
        const Matrix3 rotation = localization_->toRotationMatrix(orientation);
        const Vector3 force_body = rotation.transpose() * force_world;

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

        const milliseconds time_step = milliseconds(time_t(config_->getTimeStep() * 1E3));
        const float node_span = std::round(static_cast<float>(swing_duration.count()) / static_cast<float>(time_step.count()));
        if (node_span >= config_->getWindowSize())
        {
            std::cerr << "Node span exceeds window size" << std::endl;
            return;
        }

        const Vector3 b_pos_body_hip = robot_dynamics_->getHipLocations()[leg_id];

        Vector3 b_pos_hip_foot_0;
        legs_[leg_id]->getFootPositionEstimate(b_pos_hip_foot_0);

        const ReferenceState w_state_body = config_->getReferenceState(std::round(node_span));
        const Vector3 w_pos_N_body = w_state_body.position;
        const Vector3 w_ori_body = w_state_body.orientation;
        const Matrix3 w_R_b = localization_->toRotationMatrix(w_ori_body);
        const Vector3 w_vel_body = w_state_body.linear_velocity;

        const Vector3 w_pos_N_hip = w_pos_N_body + w_R_b * b_pos_body_hip;
        Vector3 w_pos_N_foot = w_pos_N_hip + 0.5f * w_vel_body * stance_duration.count() * 1E-3f;

        const Vector3 w_pos_hip_foot = w_pos_N_foot - w_pos_N_hip;
        const Vector3 b_pos_hip_foot_f = w_R_b.transpose() * w_pos_hip_foot;

        const Vector3 start_position = b_pos_hip_foot_0.head(3);
        const Vector3 end_position = b_pos_hip_foot_f + robot_dynamics_->getDefaultFootLocations()[leg_id];

        const Vector3 delta = end_position - start_position;
        const float radius = 0.5f * delta.norm();

        const float angle = std::atan2(delta.y(), delta.x());
        Matrix3 rotation;
        rotation = Eigen::AngleAxis<Float>(angle, Vector3::UnitZ());

        std::vector<LegCommand::Ptr> trajectory;
        for (size_t i = 0; i <= swing_resolution_; i++)
        {
            const float ratio = static_cast<float>(i) / static_cast<float>(swing_resolution_);
            const float s = M_PI * ratio;
            const float px = radius * (1 - std::cos(s));
            const float pz = step_height_ * std::sin(s);
            const Vector3 arc_position = Vector3(px, 0.0f, pz);
            const Vector3 position = rotation * arc_position + start_position;

            LegCommand::Ptr command = std::make_shared<LegCommand>();
            command->control_mode = ControlMode::POSITION;
            command->leg_id = leg_id;
            command->delay = milliseconds(time_t(swing_duration.count() * ratio * swing_duration_factor_));
            command->target_position = position;
            trajectory.push_back(command);
        }

        trajectory_publishers_[leg_id]->runTrajectory(trajectory);
    }

}