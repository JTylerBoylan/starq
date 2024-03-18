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
        QPProblem();

        /// @brief Destroy the QPProblem object
        ~QPProblem();

        /// @brief Get the MPC problem
        /// @return The MPC problem
        MPCProblem::Ptr getMPCProblem() const;

        /// @brief Update the QP problem
        /// @param config The MPC configuration
        /// @return If the update was successful
        bool update(MPCConfiguration::Ptr config);

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
        Eigen::SparseMatrix<Float> &getH();

        /// @brief Get the g vector
        /// @return The g vector
        VectorX &getG();

        /// @brief Get the Ac matrix
        /// @return The Ac matrix
        Eigen::SparseMatrix<Float> &getAc();

        /// @brief Get the lc vector
        /// @return The lc vector
        VectorX &getLc();

        /// @brief Get the uc vector
        /// @return The uc vector
        VectorX &getUc();

    protected:
        MPCProblem::Ptr mpc_problem_;

        size_t nx_;
        size_t nu_;
        size_t n_;
        size_t m_;

        Eigen::SparseMatrix<Float> H_;
        VectorX g_;
        Eigen::SparseMatrix<Float> Ac_;
        VectorX lc_;
        VectorX uc_;

        void computeH();
        void computeG();
        void computeAc();
        void computeLc();
        void computeUc();
    };

}

#endif