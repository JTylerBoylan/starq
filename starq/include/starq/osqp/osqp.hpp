#ifndef STARQ_OSQP__OSQP_HPP_
#define STARQ_OSQP__OSQP_HPP_

#include "starq/mpc/qp_solver.hpp"

#include "osqp/osqp.h"

namespace starq::osqp
{

    /// @brief OSQP class
    class OSQP : public mpc::QPSolver
    {
    public:
        using Ptr = std::shared_ptr<OSQP>;

        /// @brief Create a new OSQP object
        OSQP(const mpc::QPProblem::Ptr qp_problem);

        /// @brief Destroy the OSQP object
        ~OSQP();

        /// @brief Update the QP solver
        /// @return If the update was successful
        bool update() override;

        /// @brief Solve the QP problem
        /// @return If the problem was solved successfully
        bool solve() override;

        /// @brief Get the solution
        /// @return The solution
        starq::mpc::MPCSolution getSolution() const override;

        /// @brief Get the OSQP settings
        /// @return The OSQP settings
        OSQPSettings *getSettings() const { return settings_; }

        /// @brief Get the OSQP solver
        /// @return The OSQP solver
        OSQPSolver *getSolver() const { return solver_; }

    private:
        void convertEigenSparseToCSC(const SparseMatrix<double> &matrix,
                                     OSQPCscMatrix *&M, OSQPInt &Mnnz, OSQPFloat *&Mx, OSQPInt *&Mi, OSQPInt *&Mp);

        mpc::QPProblem::Ptr qp_problem_;

        OSQPInt n_, m_;

        OSQPSolver *solver_ = nullptr;
        OSQPSettings *settings_ = nullptr;

        OSQPFloat *q_ = nullptr;
        OSQPFloat *l_ = nullptr;
        OSQPFloat *u_ = nullptr;

        OSQPCscMatrix *P_ = nullptr;
        OSQPInt Pnnz_;
        OSQPFloat *Px_ = nullptr;
        OSQPInt *Pi_ = nullptr;
        OSQPInt *Pp_ = nullptr;

        OSQPCscMatrix *A_ = nullptr;
        OSQPInt Annz_;
        OSQPFloat *Ax_ = nullptr;
        OSQPInt *Ai_ = nullptr;
        OSQPInt *Ap_ = nullptr;
    };

}

#endif