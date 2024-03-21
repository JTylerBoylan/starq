#include "starq/mujoco/mujoco_localization.hpp"

#include <iostream>

namespace starq::mujoco
{

    MuJoCoLocalization::MuJoCoLocalization(MuJoCo::Ptr mujoco)
    {
        time_ = 0;
        position_ = Vector3::Zero();
        orientation_ = Vector3::Zero();

        mujoco->addMotorControlFunction(std::bind(&MuJoCoLocalization::localizationCallback, this,
                                                  std::placeholders::_1, std::placeholders::_2));
    }

    milliseconds MuJoCoLocalization::getCurrentTime()
    {
        return milliseconds(time_t(time_ * 1E3));
    }

    Vector3 MuJoCoLocalization::getCurrentPosition()
    {
        return position_;
    }

    Vector3 MuJoCoLocalization::getCurrentOrientation()
    {
        return orientation_;
    }

    Vector3 MuJoCoLocalization::getCurrentLinearVelocity()
    {
        return linear_velocity_;
    }

    Vector3 MuJoCoLocalization::getCurrentAngularVelocity()
    {
        return angular_velocity_;
    }

    void MuJoCoLocalization::localizationCallback(const mjModel *model, mjData *data)
    {
        (void)model; // Unused

        time_ = data->time;

        position_ = Vector3(data->qpos[0], data->qpos[1], data->qpos[2]);
        Eigen::Quaternion<Float> q(data->qpos[3], data->qpos[4], data->qpos[5], data->qpos[6]);
        quat2eul(q, orientation_);

        linear_velocity_ = Vector3(data->qvel[0], data->qvel[1], data->qvel[2]);
        angular_velocity_ = Vector3(data->qvel[3], data->qvel[4], data->qvel[5]);
    }

    void MuJoCoLocalization::quat2eul(const Eigen::Quaternion<Float> &q, Vector3 &eul)
    {
        eul.x() = atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + q.y() * q.y()));
        eul.y() = asin(2 * (q.w() * q.y() - q.z() * q.x()));
        eul.z() = atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));
    }
}