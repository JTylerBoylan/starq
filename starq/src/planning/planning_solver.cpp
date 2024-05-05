#include "starq/planning/planning_solver.hpp"

#include <iostream>
#include <fstream>

namespace starq::planning
{

    PlanSolver::PlanSolver()
    {
    }

    PlanResults::Ptr PlanSolver::solve(PlanConfiguration::Ptr config, PlanningModel::Ptr model)
    {
        config_ = config;
        model_ = model;
        results_ = std::make_shared<PlanResults>();

        const VectorX x0 = model->getInitialState();

        if (config_->dx.size() != x0.size())
        {
            std::cerr << "Invalid configuration: dx size does not match state size" << std::endl;
            results_->exit_code = ExitCode::INVALID_GRID_RESOLUTION;
            return results_;
        }

        grid_ = std::make_shared<ImplicitGrid>(config_->dx);

        if (!model->isStateValid(x0))
        {
            std::cerr << "Invalid configuration: Initial state is not valid" << std::endl;
            results_->exit_code = ExitCode::INVALID_START_STATE;
            return results_;
        }

        if (model->isStateFinal(x0))
        {
            results_->exit_code = ExitCode::START_IS_FINAL;
            return results_;
        }

        start_node_ = grid_->getNode(x0);
        start_node_->g = 0.0;
        start_node_->h = model->getHeuristic(x0);
        start_node_->f = start_node_->g + start_node_->h;

        best_node_ = start_node_;

        queue_ = std::make_shared<PriorityQueue>();
        queue_->push(start_node_);

        using namespace std::chrono;
        const auto clock_start = high_resolution_clock::now();
        while (true)
        {
            if (results_->exit_code == ExitCode::QUIT_SEARCH)
            {
                break;
            }

            if (results_->time_elapsed > config_->time_limit)
            {
                results_->exit_code = ExitCode::TIME_LIMIT;
                break;
            }

            if (results_->iterations >= config_->max_iterations)
            {
                results_->exit_code = ExitCode::ITERATION_LIMIT;
                break;
            }

            if (queue_->empty())
            {
                results_->exit_code = ExitCode::NO_NODES_IN_QUEUE;
                break;
            }

            const Node::Ptr current_node = queue_->top();
            queue_->pop();

            if (model->isStateFinal(current_node->x))
            {
                results_->exit_code = ExitCode::SUCCESS;
                best_node_ = current_node;
                break;
            }

            if (current_node->gen >= config_->max_generations)
            {
                continue;
            }

            if (current_node->h < best_node_->h)
            {
                best_node_ = current_node;
            }

            const std::vector<VectorX> actions = model->getActions(current_node->x);
            for (const VectorX &u : actions)
            {
                const Node::Ptr child = getChild(current_node, u, config_->dt);

                if (!child)
                    continue;

                const Float g = current_node->g + model->getCost(current_node->x, child->x, u, config_->dt);
                if (g < child->g)
                {
                    child->g = g;
                    updateNode(child, current_node, u);
                    queue_->push(child);
                }
            }

            results_->iterations++;

            results_->time_elapsed = duration_cast<milliseconds>(high_resolution_clock::now() - clock_start);
        }

        generatePath();

        results_->time_elapsed = duration_cast<milliseconds>(high_resolution_clock::now() - clock_start);

        return results_;
    }

    Node::Ptr PlanSolver::getChild(const Node::Ptr parent, const VectorX &u, const Float dt)
    {
        const VectorX x1 = parent->x;
        const VectorX x2 = model_->getNextState(x1, u, dt);

        if (!model_->isStateValid(x2))
            return nullptr;

        const Node::Ptr child = grid_->getNode(x2);

        if (child == parent)
            return nullptr;

        return child;
    }

    void PlanSolver::updateNode(const Node::Ptr child, const Node::Ptr parent, const VectorX &u)
    {
        if (child->h == std::numeric_limits<Float>::infinity())
        {
            child->h = model_->getHeuristic(child->x);
        }

        child->f = child->g + child->h;
        child->gen = parent->gen + 1;
        child->parent = parent;
        child->u = u;
    }

    void PlanSolver::generatePath()
    {
        results_->cost = best_node_->g;
        const size_t max_gen = best_node_->gen;
        results_->node_path.reserve(max_gen + 1);

        Node::Ptr node = best_node_;
        while (node)
        {
            results_->node_path.push_back(node);
            node = node->parent;
        }

        std::reverse(results_->node_path.begin(), results_->node_path.end());
    }

    void PlanSolver::saveNodes(const std::string &filename)
    {
        std::ofstream file(filename);
        if (!file.is_open())
        {
            std::cerr << "Failed to open file for writing: " << filename << std::endl;
            return;
        }

        for (const auto &node : grid_->getNodes())
        {
            file << node->x(0) << " " << node->x(1) << " " << node->x(2) << " "
                 << node->g << " " << node->h << " " << node->f << " "
                 << node->gen << std::endl;
        }

        file.close();
    }

}