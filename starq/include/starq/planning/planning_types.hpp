#ifndef STARQ_PLANNING__PLANNING_TYPES_HPP_
#define STARQ_PLANNING__PLANNING_TYPES_HPP_

#include <vector>
#include <memory>
#include <queue>
#include "starq/types.hpp"

namespace starq::planning
{

    enum ExitCode
    {
        SUCCESS = 0,
        RUNNING = 1,
        QUIT_SEARCH = 2,
        TIME_LIMIT = 3,
        ITERATION_LIMIT = 4,
        GENERATION_LIMIT = 5,
        NO_NODES_IN_QUEUE = 6,
        INVALID_GRID_RESOLUTION = 7,
        INVALID_START_STATE = 8
    };

    struct Node
    {
        using Ptr = std::shared_ptr<Node>;
        VectorX x, u;
        Float g = std::numeric_limits<Float>::infinity(),
              h = std::numeric_limits<Float>::infinity(),
              f = std::numeric_limits<Float>::infinity();
        uint32_t gen = 0;
        Node::Ptr parent = nullptr;
    };

    struct PlanConfiguration
    {
        using Ptr = std::shared_ptr<PlanConfiguration>;
        uint32_t max_iterations = 100000;
        uint32_t max_generations = 100;
        milliseconds time_limit = milliseconds(1000);
        Float dt = 1.0;
        VectorX dx;
    };

    struct PlanResults
    {
        using Ptr = std::shared_ptr<PlanResults>;
        milliseconds time_elapsed = milliseconds(0);
        ExitCode exit_code = RUNNING;
        uint32_t iterations = 0;
        Float cost = 0.0;
        std::vector<Node::Ptr> node_path;
    };

    struct NodeCompare
    {
        bool operator()(const Node::Ptr &lhs, const Node::Ptr &rhs) const
        {
            return lhs->f > rhs->f;
        }
    };

    using PriorityQueue = std::priority_queue<Node::Ptr, std::vector<Node::Ptr>, NodeCompare>;
    using PriorityQueue_Ptr = std::shared_ptr<PriorityQueue>;

    using GridKey = std::vector<int32_t>;

}

#endif