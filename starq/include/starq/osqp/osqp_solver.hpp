#ifndef STARQ_OSQP__OSQP_SOLVER_HPP_
#define STARQ_OSQP__OSQP_SOLVER_HPP_

#include "starq/mpc/mpc_solver.hpp"
#include "eigen3/Eigen/Sparse"

#include "osqp/osqp.h"

namespace starq::osqp
{
    using namespace starq::mpc;
    using namespace Eigen;

    class OSQP_MPCSolver : public MPCSolver
    {
    public:
        using Ptr = std::shared_ptr<OSQP_MPCSolver>;

        OSQP_MPCSolver(const MPCConfiguration &config);

        ~OSQP_MPCSolver();

        bool setup();

        MPCSolution solve() override;

        OSQPSettings *getSettings() const { return osqp_.settings; }

        OSQPSolver *getSolver() const { return osqp_.solver; }

    private:

        void setupQP();
        void calculateQPHessian();
        void calculateQPGradient();
        void calculateQPLinearConstraint();
        void calculateQPLowerBound();
        void calculateQPUpperBound();

        void setupOSQP();
        void convertEigenSparseToCSC(const SparseMatrix<double> &matrix,
                                     OSQPCscMatrix *&M, OSQPInt &Mnnz, OSQPFloat *&Mx, OSQPInt *&Mi, OSQPInt *&Mp);
        
        SparseMatrix<double> H_;
        VectorXd q_;
        SparseMatrix<double> Ac_;
        VectorXd lc_;
        VectorXd uc_;

        struct
        {
            OSQPInt n, m;

            OSQPSolver *solver = nullptr;
            OSQPSettings *settings = nullptr;

            OSQPFloat *q = nullptr;
            OSQPFloat *l = nullptr;
            OSQPFloat *u = nullptr;

            OSQPCscMatrix *P = nullptr;
            OSQPInt Pnnz;
            OSQPFloat *Px = nullptr;
            OSQPInt *Pi = nullptr;
            OSQPInt *Pp = nullptr;

            OSQPCscMatrix *A = nullptr;
            OSQPInt Annz;
            OSQPFloat *Ax = nullptr;
            OSQPInt *Ai = nullptr;
            OSQPInt *Ap = nullptr;
        } osqp_;
    };

}

#endif