#ifndef STARQ_MPC__QP_SOLVER_HPP_
#define STARQ_MPC__QP_SOLVER_HPP_

#include "starq/mpc/qp_problem.hpp"

namespace starq::mpc
{

    /// @brief QPSolver class
    class QPSolver
    {
    public:
        using Ptr = std::shared_ptr<QPSolver>;

        /// @brief Create a new QPSolver object
        /// @param qp_problem The QP problem
        QPSolver(const QPProblem::Ptr qp_problem)
            : qp_problem_(qp_problem) {}

        /// @brief Update the QP solver
        /// @return If the update was successful
        virtual bool update() = 0;

        /// @brief Solve the QP problem
        /// @return If the problem was solved successfully
        virtual bool solve() = 0;

        /// @brief Get the solution
        /// @return The solution
        virtual MPCSolution getSolution() const = 0;

    protected:
        const QPProblem::Ptr qp_problem_;
    };

}

#endif