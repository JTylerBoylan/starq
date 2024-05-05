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
        (void)u;
        (void)dt;
        return (x2.head(2) - x1.head(2)).norm();
    }

    Float STARQPlanningModel::getHeuristic(const VectorX &x1)
    {
        const VectorX dx = xf_.head(2) - x1.head(2);
        const Float dth = std::abs(std::atan2(dx(1), dx(0)) - x1(2));
        return dx.norm() + (dth > M_PI ? 2 * M_PI - dth : dth);
    }

    bool STARQPlanningModel::isStateValid(const VectorX &x)
    {
        (void)x;
        return true;
    }

    bool STARQPlanningModel::isStateFinal(const VectorX &x)
    {
        const VectorX dx = xf_ - x;
        return dx.norm() < threshold_;
    }

    std::vector<VectorX> STARQPlanningModel::getActions(const VectorX &x)
    {
        (void)x;
        std::vector<VectorX> actions(8*3, VectorX(3));
        for (int i = 0; i < 8; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                const Float vx = std::cos(i * M_PI / 4);
                const Float vy = std::sin(i * M_PI / 4);
                const Float wz = M_PI / 4 * (j - 1);

                actions[3*i + j] << vx, vy, wz;
            }
        }
        return actions;
    }

}