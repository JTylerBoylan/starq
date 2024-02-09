#include "starq/mujoco/mujoco_controller.hpp"

#include <iostream>

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
        state_.vel_ff_cmd = vel_ff * GR;
        state_.torq_ff_cmd = torque_ff / GR;
        return true;
    }

    bool MuJoCoController::setVelocity(const float vel, const float torque_ff)
    {
        const float GR = state_.gear_ratio;
        state_.vel_cmd = vel * GR;
        state_.torq_ff_cmd = torque_ff / GR;
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

    bool MuJoCoController::setGains(const float kp, const float kv)
    {
        state_.kp = kp;
        state_.kv = kv;
        return true;
    }

    void MuJoCoController::controlMotor(const mjModel *model, mjData *data)
    {

        model->actuator_gainprm[10 * motor_id_ + 0] = 1;
        model->actuator_gainprm[10 * motor_id_ + 0] = state_.kp;
        model->actuator_biasprm[10 * motor_id_ + 1] = -state_.kp;
        model->actuator_gainprm[10 * motor_id_ + 0] = state_.kv;
        model->actuator_biasprm[10 * motor_id_ + 2] = -state_.kv;

        switch (state_.control_mode)
        {
        case ControlMode::TORQUE:
            data->ctrl[motor_id_] = state_.torq_cmd;
            break;
        case ControlMode::POSITION:
            data->ctrl[motor_id_] = state_.pos_cmd;
            break;
        case ControlMode::VELOCITY:
            data->ctrl[motor_id_] = state_.vel_cmd;
            break;
        }

        state_.pos_est = data->qpos[motor_id_];
        state_.vel_est = data->qvel[motor_id_];
        state_.torq_est = data->qfrc_actuator[motor_id_];

        std::cout << "pos: " << state_.pos_est << " vel: " << state_.vel_est << " torq: " << state_.torq_est << std::endl;
    }

}