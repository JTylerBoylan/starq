#ifndef STARQ_PLANNING__PLANNING_MODEL_HPP_
#define STARQ_PLANNING__PLANNING_MODEL_HPP_

#include "starq/planning/planning_types.hpp"

namespace starq::planning
{

    class PlanningModel
    {
        public:
            using Ptr = std::shared_ptr<PlanningModel>;

            virtual VectorX getInitialState() = 0;

            virtual VectorX getNextState(const VectorX &x1, const VectorX &u, const Float dt) = 0;

            virtual Float getCost(const VectorX &x1, const VectorX &x2, const VectorX &u, const Float dt) = 0;

            virtual Float getHeuristic(const VectorX &x1) = 0;

            virtual bool isStateValid(const VectorX &x) = 0;

            virtual bool isStateFinal(const VectorX &x) = 0;

            virtual std::vector<VectorX> getActions(const VectorX &x, const VectorX &dx, const Float dt) = 0;
    };

}

#endif