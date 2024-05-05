#ifndef STARQ_PLANNING__PLANNING_SOLVER_HPP_
#define STARQ_PLANNING__PLANNING_SOLVER_HPP_

#include "starq/planning/planning_types.hpp"
#include "starq/planning/planning_model.hpp"
#include "starq/planning/implicit_grid.hpp"

namespace starq::planning
{

    class PlanSolver
    {
    public:
        using Ptr = std::shared_ptr<PlanSolver>;

        PlanSolver();

        PlanResults::Ptr solve(PlanConfiguration::Ptr config, PlanningModel::Ptr model);

        ImplicitGrid::Ptr getGrid() const { return grid_; }

        PriorityQueue_Ptr getQueue() const { return queue_; }

        void saveNodes(const std::string &filename);

    private:

        Node::Ptr getChild(const Node::Ptr parent, const VectorX &u, const Float dt);

        void updateNode(const Node::Ptr node, const Node::Ptr parent, const VectorX &u);

        void generatePath();

        ImplicitGrid::Ptr grid_;
        PriorityQueue_Ptr queue_;
        PlanResults::Ptr results_;
        PlanConfiguration::Ptr config_;
        PlanningModel::Ptr model_;

        Node::Ptr start_node_, best_node_;
    };

}

#endif