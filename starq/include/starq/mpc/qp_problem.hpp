#ifndef STARQ_MPC__QP_PROBLEM_HPP_
#define STARQ_MPC__QP_PROBLEM_HPP_

#include "starq/mpc/mpc_problem.hpp"
#include "eigen3/Eigen/Sparse"

namespace starq::mpc
{

    /// @brief QPProblem class
    class QPProblem
    {
    public:
        using Ptr = std::shared_ptr<QPProblem>;

        /// @brief Create a new QPProblem object
        /// @param mpc_problem MPC problem
        QPProblem(const MPCProblem::Ptr mpc_problem);

        /// @brief Destroy the QPProblem object
        ~QPProblem();

        /// @brief Get the MPC problem
        /// @return The MPC problem
        MPCProblem::Ptr getMPCProblem() const;

        /// @brief Update the QP problem
        void update();

        /// @brief Get the number of states
        /// @return The number of states
        size_t getNx() const;

        /// @brief Get the number of inputs
        /// @return The number of inputs
        size_t getNu() const;

        /// @brief Get the number of total states
        /// @return The number of total states
        size_t getN() const;

        /// @brief Get the number of total constraints
        /// @return The number of total constraints
        size_t getM() const;

        /// @brief Get the H matrix
        /// @return The H matrix
        SparseMatrix<double> &getH();

        /// @brief Get the g vector
        /// @return The g vector
        VectorXd &getG();

        /// @brief Get the Ac matrix
        /// @return The Ac matrix
        SparseMatrix<double> &getAc();

        /// @brief Get the lc vector
        /// @return The lc vector
        VectorXd &getLc();

        /// @brief Get the uc vector
        /// @return The uc vector
        VectorXd &getUc();

    protected:
        const MPCProblem::Ptr mpc_problem_;

        size_t n_;
        size_t m_;

        size_t nx_;
        size_t nu_;

        SparseMatrix<double> H_;
        VectorXd g_;
        SparseMatrix<double> Ac_;
        VectorXd lc_;
        VectorXd uc_;

        void computeH();
        void computeG();
        void computeAc();
        void computeLc();
        void computeUc();
    };

}

#endif