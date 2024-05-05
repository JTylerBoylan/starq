#ifndef STARQ_TESTING__DUMMY_PLANNING_MODEL_HPP_
#define STARQ_TESTING__DUMMY_PLANNING_MODEL_HPP_

#include "starq/planning/planning_model.hpp"

namespace starq::testing
{

    class DummyPlanningModel : public planning::PlanningModel
    {
    public:
        using Ptr = std::shared_ptr<DummyPlanningModel>;

        DummyPlanningModel() {}

        VectorX getInitialState() override
        {
            VectorX x(2);
            x << 10.0, 10.0;
            return x;
        }

        VectorX getNextState(const VectorX &x1, const VectorX &u, const Float dt) override
        {
            VectorX x(2);
            x << x1(0) + u(0) * dt, x1(1) + u(1) * dt;
            return x;
        }

        Float getCost(const VectorX &x1, const VectorX &x2, const VectorX &u, const Float dt) override
        {
            (void)u;
            (void)dt;
            return (x2 - x1).norm();
        }

        Float getHeuristic(const VectorX &x1) override
        {
            return x1.norm();
        }

        bool isStateValid(const VectorX &x) override
        {
            (void)x;
            return true;
        }

        bool isStateFinal(const VectorX &x) override
        {
            return x.norm() < 0.1;
        }

        std::vector<VectorX> getActions(const VectorX &x) override
        {
            (void)x;
            std::vector<VectorX> actions(8, VectorX(2));
            for (size_t i = 0; i < 8; i++)
            {
                actions[i] << std::cos(i * M_PI / 4), std::sin(i * M_PI / 4);
            }
            return actions;
        }
    };

}

#endif