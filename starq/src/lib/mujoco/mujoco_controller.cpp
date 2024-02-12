#include "starq/mujoco/mujoco_controller.hpp"

#include <iostream>

#define MUJOCO_Q_OFFSET 7

namespace starq::mujoco
{

    MuJoCoController::MuJoCoController(MuJoCo::Ptr mujoco, const int motor_id)
        : mujoco_(mujoco), motor_id_(motor_id)
    {
        mujoco_->addMotorControlFunction(std::bind(&MuJoCoController::controlMotor, this,
                                                   std::placeholders::_1, std::placeholders::_2));
    }

    bool MuJoCoController::setGearRatio(const float gear_ratio)
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

    bool MuJoCoController::setPosition(const float pos, const float vel_ff, const float torque_ff)
    {
        const float GR = state_.gear_ratio;
        state_.pos_cmd = pos * GR;
        state_.vel_cmd = vel_ff * GR;
        state_.torq_cmd = torque_ff / GR;
        return true;
    }

    bool MuJoCoController::setVelocity(const float vel, const float torque_ff)
    {
        const float GR = state_.gear_ratio;
        state_.vel_cmd = vel * GR;
        state_.torq_cmd = torque_ff / GR;
        return true;
    }

    bool MuJoCoController::setTorque(const float torque)
    {
        const float GR = state_.gear_ratio;
        state_.torq_cmd = torque / GR;
        return true;
    }

    float MuJoCoController::getPositionEstimate()
    {
        return state_.pos_est;
    }

    float MuJoCoController::getVelocityEstimate()
    {
        return state_.vel_est;
    }

    float MuJoCoController::getTorqueEstimate()
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
        const mjtNum vel_est = data->qvel[motor_id_ + MUJOCO_Q_OFFSET];
        const mjtNum torq_est = data->qfrc_applied[motor_id_ + MUJOCO_Q_OFFSET];

        const mjtNum kp = state_.kp;
        const mjtNum kv = state_.kd;
        const mjtNum ki = state_.ki;

        const mjtNum pos_err = state_.pos_cmd - pos_est;
        const mjtNum vel_err = state_.vel_cmd - vel_est;

        state_.torq_integral += pos_err * ki;

        const mjtNum torq_cmd = state_.torq_cmd + pos_err * kp + vel_err * kv + state_.torq_integral;

        data->ctrl[motor_id_] = torq_cmd;

        state_.pos_est = pos_est;
        state_.vel_est = vel_est;
        state_.torq_est = torq_est;

        // std::cout << "pos_cmd: " << pos_cmd << " pos_est: " << pos_est
        //           << " vel_cmd: " << vel_cmd << " vel_est: " << vel_est
        //           << " torq_cmd: " << torq_cmd << " torq_est: " << torq_est << std::endl;
    }

}