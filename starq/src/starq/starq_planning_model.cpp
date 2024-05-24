#include "starq/starq/starq_planning_model.hpp"

namespace starq
{

    STARQPlanningModel::STARQPlanningModel(slam::Localization::Ptr localization)
        : localization_(localization)
    {
        xf_ = VectorX::Zero(3);
        threshold_ = 0.25;
        dx_ = Vector3(0.05, 0.05, M_PI / 64.0);
        dt_ = 0.15;
    }

    void STARQPlanningModel::setGoalState(const VectorX &xf)
    {
        xf_ = xf;
        computeActions();
    }

    void STARQPlanningModel::setGoalThreshold(const Float threshold)
    {
        threshold_ = threshold;
        computeActions();
    }

    void STARQPlanningModel::setGridResolution(const VectorX &dx)
    {
        dx_ = dx;
        computeActions();
    }

    void STARQPlanningModel::setTimeStep(const Float dt)
    {
        dt_ = dt;
        computeActions();
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
        const Float cx = 1.0;
        const Float cth = 0.25;
        const VectorX dx = xf_ - x;
        const Float dth = xf_(2) - x(2);
        return cx * dx.norm() + cth * (dth > M_PI ? 2 * M_PI - dth : dth);
    }

    bool STARQPlanningModel::isStateValid(const VectorX &x)
    {
        (void)x;
        return true;
    }

    bool STARQPlanningModel::isStateFinal(const VectorX &x)
    {
        return getHeuristic(x) < threshold_;
    }

    std::vector<VectorX> STARQPlanningModel::getActions(const VectorX &x)
    {
        (void)x;
        return actions_;
    }

    void STARQPlanningModel::computeActions()
    {
        const int res_v = 8;
        const int res_w = 7;

        const VectorX v_mag = dx_ / dt_;
        const VectorX v_fac = Vector3(2.0, 1.0, 2.0);
        const VectorX v = v_mag.cwiseProduct(v_fac);
        const Float vx_off = 0.25;

        const Float dth = 2 * M_PI / res_v;
        const int w_off = res_w / 2;

        actions_.resize(res_v * res_w, Vector3());
        for (int i = 0; i < res_v; i++)
        {
            for (int j = 0; j < res_w; j++)
            {
                const Float vx = v.x() * std::cos(i * dth) + vx_off;
                const Float vy = v.y() * std::sin(i * dth);
                const Float wz = v.z() * (j - w_off);

                actions_[res_w * i + j] << vx, vy, wz;
            }
        }
    }

}