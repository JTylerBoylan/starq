#include "starq/starq/starq_planning_model.hpp"

#include <iostream>

namespace starq
{

    STARQPlanningModel::STARQPlanningModel(slam::Localization::Ptr localization)
        : localization_(localization)
    {
        x_goal_ = VectorX::Zero(3);
        v_max_ = Vector3(1.0, 1.0, M_PI / 8.0);
        rho_t_ = 0.25;
        N_ = 10;
        g_ = 5;
        v_fac_ = Vector3(1.0, 0.5, 1.0);
        v_off_ = Vector3(0.0, 0.0, 0.0);
    }

    void STARQPlanningModel::setGoalState(const VectorX &xf)
    {
        x_goal_ = xf;
    }

    void STARQPlanningModel::setMaxVelocity(const VectorX &v_max)
    {
        v_max_ = v_max;
    }

    void STARQPlanningModel::setDampingRadius(const Float rho_t)
    {
        rho_t_ = rho_t;
    }

    void STARQPlanningModel::setIdealNodeCount(const Float N)
    {
        N_ = N;
    }

    void STARQPlanningModel::setGridResolutionFactor(const Float g)
    {
        g_ = g;
    }

    void STARQPlanningModel::setVelocityFactor(const VectorX &v_fac)
    {
        v_fac_ = v_fac;
    }

    void STARQPlanningModel::setVelocityOffset(const VectorX &v_off)
    {
        v_off_ = v_off;
    }

    VectorX STARQPlanningModel::getVelocity()
    {
        return v_;
    }

    Float STARQPlanningModel::getTimeStep()
    {
        return dt_;
    }

    VectorX STARQPlanningModel::getGridResolution()
    {
        return dx_;
    }

    Float STARQPlanningModel::getGoalThreshold()
    {
        return threshold_;
    }

    void STARQPlanningModel::update(PlanConfiguration::Ptr config)
    {
        computeActions();

        config->dx = dx_;
        config->dt = dt_;
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

        const Float theta2 = x1(2) + wz * dt;
        const Float x2 = x1(0) + vx * std::cos(theta2) * dt - vy * std::sin(theta2) * dt;
        const Float y2 = x1(1) + vx * std::sin(theta2) * dt + vy * std::cos(theta2) * dt;

        return wrap(Vector3(x2, y2, theta2));
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
        const VectorX dx = wrap(x_goal_ - x);
        const Float dt = dx.cwiseQuotient(v_).norm();
        return dt;
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
        const int res_w = 3;

        const VectorX delta_x = wrap(x_goal_ - getInitialState());
        v_ = v_max_.cwiseMin((delta_x.cwiseQuotient(v_max_).norm() / rho_t_) * v_max_);

        dt_ = delta_x.cwiseQuotient(v_).norm() / N_;
        dx_ = (dt_ / (g_ * std::sqrt(2))) * v_;
        threshold_ = dt_;

        const Float dth = 2 * M_PI / res_v;
        const int w_off = res_w / 2;

        actions_.resize(res_v * res_w, Vector3());
        for (int i = 0; i < res_v; i++)
        {
            for (int j = 0; j < res_w; j++)
            {
                const Float vx = v_.x() * std::cos(i * dth) * v_fac_.x() + v_off_.x();
                const Float vy = v_.y() * std::sin(i * dth) * v_fac_.y() + v_off_.y();
                const Float wz = v_.z() * (j - w_off);

                actions_[res_w * i + j] << vx, vy, wz;
            }
        }
    }

    VectorX STARQPlanningModel::wrap(const VectorX &x)
    {
        VectorX x_wrap = x;
        x_wrap(2) = std::fmod(x(2), 2 * M_PI);
        if (x_wrap(2) > M_PI)
        {
            x_wrap(2) -= 2 * M_PI;
        }
        else if (x_wrap(2) < -M_PI)
        {
            x_wrap(2) += 2 * M_PI;
        }
        return x_wrap;
    }

}