#ifndef STARQ__TYPES_HPP_
#define STARQ__TYPES_HPP_

#include <chrono>
#include "eigen3/Eigen/Dense"

namespace starq
{
    using milliseconds = std::chrono::milliseconds;

    using Float = double;

    using Vector3 = Eigen::Matrix<Float, 3, 1>;
    using VectorX = Eigen::Matrix<Float, -1, 1>;

    using Matrix3 = Eigen::Matrix<Float, 3, 3>;
    using MatrixX = Eigen::Matrix<Float, -1, -1>;

    enum AxisState
    {
        UNDEFINED = 0x0,
        IDLE = 0x1,
        CLOSED_LOOP_CONTROL = 0x8
    };

    enum ControlMode
    {
        VOLTAGE = 0x0,
        TORQUE = 0x1,
        VELOCITY = 0x2,
        POSITION = 0x3
    };

    enum InputMode
    {
        INACTIVE = 0x0,
        PASSTHROUGH = 0x1,
        VEL_RAMP = 0x2,
        POS_FILTER = 0x3,
        TRAP_TRAJ = 0x5,
        TORQUE_RAMP = 0x6
    };
}

#endif