#ifndef STARQ_STARQ__PLANNING_MODEL_HPP_
#define STARQ_STARQ__PLANNING_MODEL_HPP_

#include "starq/planning/planning_model.hpp"
#include "starq/slam/localization.hpp"

namespace starq
{
    using namespace starq::planning;

    class STARQPlanningModel : public PlanningModel
    {
    public:
        using Ptr = std::shared_ptr<STARQPlanningModel>;

        STARQPlanningModel(slam::Localization::Ptr localization);

        void setGoalState(const VectorX &xf);

        void setMaxVelocity(const VectorX &v_max);

        void setDampingRadius(const Float rho_t);

        void setIdealNodeCount(const Float N);

        void setGridResolutionFactor(const Float g);

        void setVelocityFactor(const VectorX &v_fac);

        void setVelocityOffset(const VectorX &v_off);

        VectorX getVelocity();

        Float getTimeStep();

        VectorX getGridResolution();

        Float getGoalThreshold();

        void update(PlanConfiguration::Ptr config) override;

        VectorX getInitialState() override;

        VectorX getNextState(const VectorX &x1, const VectorX &u, const Float dt) override;

        Float getCost(const VectorX &x1, const VectorX &x2, const VectorX &u, const Float dt) override;

        Float getHeuristic(const VectorX &x) override;

        bool isStateValid(const VectorX &x) override;

        bool isStateFinal(const VectorX &x) override;

        std::vector<VectorX> getActions(const VectorX &x) override;

    private:
        void computeActions();

        VectorX wrap(const VectorX &x);

        slam::Localization::Ptr localization_;

        VectorX x_goal_;
        VectorX v_max_;
        Float rho_t_;
        Float N_;
        Float g_;
        VectorX v_fac_;
        VectorX v_off_;

        VectorX v_;
        Float dt_;
        VectorX dx_;
        Float threshold_;

        std::vector<VectorX> actions_;
    };

}

#endif