#include "starq/mujoco/mujoco_controller.hpp"

#include <iostream>

#define MUJOCO_Q_OFFSET 7
#define MUJOCO_V_OFFSET 6

namespace starq::mujoco
{

    MuJoCoController::MuJoCoController(MuJoCo::Ptr mujoco, const int motor_id)
        : mujoco_(mujoco), motor_id_(motor_id)
    {
        mujoco_->addMotorControlFunction(std::bind(&MuJoCoController::controlMotor, this,
                                                   std::placeholders::_1, std::placeholders::_2));
    }

    bool MuJoCoController::setGearRatio(const Float gear_ratio)
    {
        state_.gear_ratio = gear_ratio;
        return true;
    }

    bool MuJoCoController::setState(const uint32_t state)
    {
        state_.state = state;
        return true;
    }

    bool MuJoCoController::setControlMode(const uint32_t control_mode, const uint32_t input_mode)
    {
        state_.control_mode = control_mode;
        state_.input_mode = input_mode;
        return true;
    }

    bool MuJoCoController::setPosition(const Float pos, const Float vel_ff, const Float torque_ff)
    {
        const Float GR = state_.gear_ratio;
        state_.pos_cmd = pos * GR;
        state_.vel_cmd = vel_ff * GR;
        state_.torq_cmd = torque_ff / GR;
        return true;
    }

    bool MuJoCoController::setVelocity(const Float vel, const Float torque_ff)
    {
        const Float GR = state_.gear_ratio;
        state_.vel_cmd = vel * GR;
        state_.torq_cmd = torque_ff / GR;
        return true;
    }

    bool MuJoCoController::setTorque(const Float torque)
    {
        const Float GR = state_.gear_ratio;
        state_.torq_cmd = torque / GR;
        return true;
    }

    Float MuJoCoController::getPositionEstimate()
    {
        return state_.pos_est;
    }

    Float MuJoCoController::getVelocityEstimate()
    {
        return state_.vel_est;
    }

    Float MuJoCoController::getTorqueEstimate()
    {
        return state_.torq_est;
    }

    bool MuJoCoController::setGains(const float kp, const float kd, const float ki)
    {
        state_.kp = kp;
        state_.kd = kd;
        state_.ki = ki;
        return true;
    }

    void MuJoCoController::controlMotor(const mjModel *model, mjData *data)
    {
        (void)model;

        const mjtNum pos_est = data->qpos[motor_id_ + MUJOCO_Q_OFFSET];
        const mjtNum vel_est = data->qvel[motor_id_ + MUJOCO_V_OFFSET];
        const mjtNum torq_est = data->ctrl[motor_id_];

        switch (state_.control_mode)
        {
        case ControlMode::POSITION:
        {
            const mjtNum pos_err = state_.pos_cmd - pos_est;
            const mjtNum vel_err = state_.vel_cmd - vel_est;

            const mjtNum kp = state_.kp;
            const mjtNum kd = state_.kd;
            const mjtNum ki = state_.ki;
            state_.torq_integral += pos_err * ki;
            const mjtNum torq_cmd = state_.torq_cmd + pos_err * kp + vel_err * kd + state_.torq_integral;
            data->ctrl[motor_id_] = torq_cmd;
            break;
        }
        case ControlMode::VELOCITY:
        {
            const mjtNum vel_err = state_.vel_cmd - vel_est;

            const mjtNum kp = state_.kp;
            const mjtNum torq_cmd = state_.torq_cmd + vel_err * kp;
            data->ctrl[motor_id_] = torq_cmd;
            break;
        }
        case ControlMode::TORQUE:
        {
            data->ctrl[motor_id_] = state_.torq_cmd;
            break;
        }
        }

        state_.pos_est = pos_est;
        state_.vel_est = vel_est;
        state_.torq_est = torq_est;

        // std::cout << "pos_cmd: " << pos_cmd << " pos_est: " << pos_est
        //           << " vel_cmd: " << vel_cmd << " vel_est: " << vel_est
        //           << " torq_cmd: " << torq_cmd << " torq_est: " << torq_est << std::endl;
    }

}