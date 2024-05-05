#include "starq/starq/starq_planning_model.hpp"

namespace starq
{

    STARQPlanningModel::STARQPlanningModel(slam::Localization::Ptr localization)
        : localization_(localization)
    {
        xf_ = VectorX::Zero(3);
        threshold_ = 0.25;
    }

    void STARQPlanningModel::setGoalState(const VectorX &xf)
    {
        xf_ = xf;
    }

    void STARQPlanningModel::setGoalThreshold(const Float threshold)
    {
        threshold_ = threshold;
    }

    VectorX STARQPlanningModel::getInitialState()
    {
        const Vector3 position = localization_->getCurrentPosition();
        const Vector3 orientation = localization_->getCurrentOrientation();

        return Vector3(position.x(), position.y(), orientation.z());
    }

    VectorX STARQPlanningModel::getNextState(const VectorX &x1, const VectorX &u, const Float dt)
    {
        const Float vx = u(0);
        const Float vy = u(1);
        const Float wz = u(2);

        Float theta2 = x1(2) + wz * dt;
        const Float x2 = x1(0) + vx * std::cos(theta2) * dt - vy * std::sin(theta2) * dt;
        const Float y2 = x1(1) + vx * std::sin(theta2) * dt + vy * std::cos(theta2) * dt;

        if (theta2 > M_PI)
        {
            theta2 -= 2 * M_PI;
        }
        else if (theta2 < -M_PI)
        {
            theta2 += 2 * M_PI;
        }

        return Vector3(x2, y2, theta2);
    }

    Float STARQPlanningModel::getCost(const VectorX &x1, const VectorX &x2, const VectorX &u, const Float dt)
    {
        (void)x1;
        (void)x2;
        (void)u;
        return dt;
    }

    Float STARQPlanningModel::getHeuristic(const VectorX &x)
    {
        const VectorX dx = xf_.head(2) - x.head(2);
        return dx.norm();
    }

    bool STARQPlanningModel::isStateValid(const VectorX &x)
    {
        (void)x;
        return true;
    }

    bool STARQPlanningModel::isStateFinal(const VectorX &x)
    {
        const VectorX dx = (xf_.head(2) - x.head(2));
        return dx.norm() < threshold_;
    }

    std::vector<VectorX> STARQPlanningModel::getActions(const VectorX &x)
    {
        (void)x;
        int res_v = 8;
        int res_w = 7;
        Float vx_mag = 0.5;
        Float vx_off = 0.25;
        Float vy_mag = 0.25;
        Float dth = 2 * M_PI / res_v;
        Float dw = M_PI / 16;
        int w_off = res_w / 2;
        std::vector<VectorX> actions(res_v * res_w, Vector3());
        for (int i = 0; i < res_v; i++)
        {
            for (int j = 0; j < res_w; j++)
            {
                const Float vx = vx_mag * std::cos(i * dth) + vx_off;
                const Float vy = vy_mag * std::sin(i * dth);
                const Float wz = dw * (j - w_off);

                actions[res_w * i + j] << vx, vy, wz;
            }
        }
        return actions;
    }

}