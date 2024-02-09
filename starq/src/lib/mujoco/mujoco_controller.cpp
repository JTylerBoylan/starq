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

    bool MuJoCoController::setGains(const float pos_gain, const float vel_gain, const float integrator_gain)
    {
        state_.pos_gain = pos_gain;
        state_.vel_gain = vel_gain;
        state_.int_gain = integrator_gain;
        return true;
    }

    void MuJoCoController::controlMotor(const mjModel *model, mjData *data)
    {
        (void)model;

        const mjtNum pos_est = data->qpos[motor_id_];
        const mjtNum vel_est = data->qvel[motor_id_];
        const mjtNum torq_est = data->act[motor_id_];

        mjtNum pos_cmd = state_.pos_cmd;
        mjtNum vel_cmd = state_.vel_cmd;
        mjtNum torq_cmd = state_.torq_cmd;

        const mjtNum vel_ff_cmd = state_.vel_ff_cmd;
        const mjtNum torq_ff_cmd = state_.torq_ff_cmd;

        const mjtNum pos_gain = state_.pos_gain;
        const mjtNum vel_gain = state_.vel_gain;
        const mjtNum int_gain = state_.int_gain;

        switch (state_.control_mode)
        {
        case ControlMode::POSITION:
        {
            const mjtNum pos_error = pos_cmd - pos_est;
            vel_cmd = pos_error * pos_gain + vel_ff_cmd;
            [[fallthrough]];
        }
        case ControlMode::VELOCITY:
        {
            const mjtNum vel_error = vel_cmd - vel_est;
            state_.torq_integral += vel_error * int_gain;
            torq_cmd = vel_error * vel_gain + state_.torq_integral + torq_ff_cmd;
        }
        }

        data->ctrl[motor_id_] = torq_cmd;

        state_.pos_est = pos_est;
        state_.vel_est = vel_est;
        state_.torq_est = torq_est;

        std::cout << "pos_cmd: " << pos_cmd << " pos_est: " << pos_est
                  << " vel_cmd: " << vel_cmd << " vel_est: " << vel_est
                  << " torq_cmd: " << torq_cmd << " torq_est: " << torq_est << std::endl;
    }

}