#ifndef STARQ_MPC__QP_SOLVER_HPP_
#define STARQ_MPC__QP_SOLVER_HPP_

#include "starq/mpc/qp_problem.hpp"

namespace starq::mpc
{

    /// @brief MPCSolver class
    class MPCSolver
    {
    public:
        using Ptr = std::shared_ptr<MPCSolver>;

        /// @brief Update the MPCSolver solver
        /// @param config The MPC configuration
        /// @return If the update was successful
        virtual bool update(MPCConfiguration::Ptr config) = 0;

        /// @brief Solve the MPC problem
        /// @return If the problem was solved successfully
        virtual bool solve() = 0;

        /// @brief Get the solution
        /// @return The solution
        virtual MPCSolution getSolution() const = 0;

        /// @brief Get the first force state of the solution
        /// @return The first force state
        virtual FootForceState getFirstForceState() const = 0;

    };

}

#endif