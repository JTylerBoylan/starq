#include "starq/mpc/mpc_controller.hpp"

#include <iostream>

namespace starq::mpc
{

    MPCController::MPCController(const MPCConfiguration::Ptr config,
                                 const MPCSolver::Ptr solver,
                                 const slam::Localization::Ptr localization,
                                 const RobotDynamics::Ptr robot_dynamics,
                                 const LegCommandPublisher::Ptr leg_command_publisher,
                                 const TrajectoryPublisher::Ptr trajectory_publisher)
        : config_(config),
          solver_(solver),
          localization_(localization),
          robot_dynamics_(robot_dynamics),
          leg_command_publisher_(leg_command_publisher),
          trajectory_publisher_(trajectory_publisher),
          stop_on_fail_(false),
          sleep_duration_us_(1000)
    {
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
            if (!solver_->update(config_) && stop_on_fail_)
                break;

            if (!solver_->solve() && stop_on_fail_)
                break;

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

            std::this_thread::sleep_for(sleep_duration_us_);
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

        const Vector3f position_world = localization_->getCurrentPosition();
        const Vector3f orientation_world = localization_->getCurrentOrientation();
        const Matrix3f rotation_world = localization_->toRotationMatrix(orientation_world);
        const Vector3f velocity_world = config_->getReferenceState(0).linear_velocity;

        const Vector3f hip_position_body = robot_dynamics_->getHipLocations()[leg_id];
        const Vector3f hip_position_world = position_world + rotation_world * hip_position_body;

        VectorXf start_position_hip;
        leg_command_publisher_->getLegControllers()[leg_id]->getFootPositionEstimate(start_position_hip);
        const Vector3f start_position_world = hip_position_world + rotation_world * start_position_hip;
        Vector3f end_position_world = hip_position_world + velocity_world * stance_duration.count() * 1E-3f;
        end_position_world.z() = 0.0;

        


    }

}