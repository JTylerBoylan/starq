#ifndef STARQ_MPC__MPC_PROBLEM_HPP_
#define STARQ_MPC__MPC_PROBLEM_HPP_

#include "starq/mpc/mpc_configuration.hpp"

namespace starq::mpc
{

    /// @brief MPCProblem class
    class MPCProblem
    {
    public:
        using Ptr = std::shared_ptr<MPCProblem>;

        /// @brief Create a new MPCProblem object
        MPCProblem();

        /// @brief Destroy the MPCProblem object
        ~MPCProblem();

        /// @brief Update the MPC problem
        /// @param config The MPC configuration
        /// @return If the update was successful
        bool update(MPCConfiguration::Ptr config);

        /// @brief Get the MPC configuration
        MPCConfiguration::Ptr getConfig() const;

        /// @brief Get the reference state at node k
        /// @param k The node index
        /// @return The reference state vector
        VectorXf &getXref(const size_t &k);

        /// @brief Get the Q matrix at node k
        /// @param k The node index
        /// @return The Q matrix
        MatrixXf &getQ(const size_t &k);

        /// @brief Get the R matrix at node k
        /// @param k The node index
        /// @return The R matrix
        MatrixXf &getR(const size_t &k);

        /// @brief Get the A matrix at node k
        /// @param k The node index
        /// @return The A matrix
        MatrixXf &getA(const size_t &k);

        /// @brief Get the B matrix at node k
        /// @param k The node index
        /// @return The B matrix
        MatrixXf &getB(const size_t &k);

        /// @brief Get the C matrix at node k
        /// @param k The node index
        /// @return The C matrix
        MatrixXf &getC(const size_t &k);

        /// @brief Get the lower bound vector at node k
        /// @param k The node index
        /// @return The lower bound vector
        VectorXf &getLower(const size_t &k);

        /// @brief Get the upper bound vector at node k
        /// @param k The node index
        /// @return The upper bound vector
        VectorXf &getUpper(const size_t &k);

    private:
        MPCConfiguration::Ptr config_;

        std::vector<VectorXf> xref_;
        std::vector<MatrixXf> Q_;
        std::vector<MatrixXf> R_;
        std::vector<MatrixXf> A_;
        std::vector<MatrixXf> B_;
        std::vector<MatrixXf> C_;
        std::vector<VectorXf> lower_;
        std::vector<VectorXf> upper_;

        void computeXref();
        void computeQ();
        void computeR();
        void computeA();
        void computeB();
        void computeC();
        void computeLower();
        void computeUpper();

        Matrix3f getSkewSymmetricMatrix(const Vector3f &v) const;
    };

}

#endif