#ifndef STARQ_OSQP__OSQP_HPP_
#define STARQ_OSQP__OSQP_HPP_

#include "starq/mpc/qp_problem.hpp"

#include "osqp/osqp.h"

namespace starq::osqp
{

    class OSQP
    {
    public:
        using Ptr = std::shared_ptr<OSQP>;

        OSQP(const mpc::QPProblem::Ptr qp_problem);

        ~OSQP();

        void update();

        void solve();

        OSQPSettings *getSettings() const { return settings_; }

        OSQPSolver *getSolver() const { return solver_; }

        starq::mpc::MPCSolution getSolution() const;

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