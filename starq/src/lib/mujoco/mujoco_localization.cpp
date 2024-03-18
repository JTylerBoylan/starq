#include "starq/mujoco/mujoco_localization.hpp"

namespace starq::mujoco
{

    MuJoCoLocalization::MuJoCoLocalization(MuJoCo::Ptr mujoco)
    {
        position_ = Vector3::Zero();
        orientation_ = Vector3::Zero();
        last_position_ = Vector3::Zero();
        last_orientation_ = Vector3::Zero();
        last_time_ = 0;

        mujoco->addMotorControlFunction(std::bind(&MuJoCoLocalization::localizationCallback, this,
                                                  std::placeholders::_1, std::placeholders::_2));
    }

    milliseconds MuJoCoLocalization::getCurrentTime()
    {
        return milliseconds(time_t(last_time_ * 1E3));
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

        last_position_ = position_;
        last_orientation_ = orientation_;

        auto now = data->time;
        auto elapsed_time = now - last_time_;
        last_time_ = now;

        position_ = Vector3(data->qpos[0], data->qpos[1], data->qpos[2]);

        Eigen::Quaternion<Float> q(data->qpos[3], data->qpos[4], data->qpos[5], data->qpos[6]);
        quat2eul(q, orientation_);

        linear_velocity_ = (position_ - last_position_) / elapsed_time;
        angular_velocity_ = (orientation_ - last_orientation_) / elapsed_time;
    }

    void MuJoCoLocalization::quat2eul(const Eigen::Quaternion<Float> &q, Vector3 &eul)
    {
        eul.x() = atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + q.y() * q.y()));
        eul.y() = asin(2 * (q.w() * q.y() - q.z() * q.x()));
        eul.z() = atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));
    }
}