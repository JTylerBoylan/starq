#ifndef STARQ_STARQ__PLANNING_MODEL_HPP_
#define STARQ_STARQ__PLANNING_MODEL_HPP_

#include "starq/planning/planning_model.hpp"
#include "starq/slam/localization.hpp"

namespace starq
{

    class STARQPlanningModel : public planning::PlanningModel
    {
    public:
        using Ptr = std::shared_ptr<STARQPlanningModel>;

        STARQPlanningModel(slam::Localization::Ptr localization);

        void setGoalState(const VectorX &xf);

        void setGoalThreshold(const Float threshold);

        VectorX getInitialState() override;

        VectorX getNextState(const VectorX &x1, const VectorX &u, const Float dt) override;

        Float getCost(const VectorX &x1, const VectorX &x2, const VectorX &u, const Float dt) override;

        Float getHeuristic(const VectorX &x) override;

        bool isStateValid(const VectorX &x) override;

        bool isStateFinal(const VectorX &x) override;

        std::vector<VectorX> getActions(const VectorX &x, const VectorX &dx, const Float dt) override;

    private:
        slam::Localization::Ptr localization_;
        VectorX xf_;
        Float threshold_;
    };

}

#endif