#include "starq/osqp/osqp_solver.hpp"

#include <iostream>

namespace starq::osqp
{

    OSQP_MPCSolver::OSQP_MPCSolver(const MPCConfiguration &config)
        : MPCSolver(config)
    {
        const int total_x = 13 * (config.window_size + 1);
        int total_u;
        for (auto &n : n_legs_)
            total_u = n * 3;

        num_vars_ = total_x + total_u;
        num_constraints_ = 2 * total_x + total_u;

        setupQP();
        setupOSQP();
    }

    OSQP_MPCSolver::~OSQP_MPCSolver()
    {
        osqp_cleanup(osqp_.solver);
        delete osqp_.P;
        delete osqp_.A;
        delete osqp_.settings;
    }

    bool OSQP_MPCSolver::solve()
    {
        return osqp_solve(osqp_.solver) == 0;
    }

    void OSQP_MPCSolver::setupQP()
    {
        H_.resize(num_vars_, num_vars_);
        q_.resize(num_vars_);
        Ac_.resize(num_constraints_, num_vars_);
        lc_.resize(num_constraints_);
        uc_.resize(num_constraints_);

        calculateQPHessian();
        calculateQPGradient();
        calculateQPLinearConstraint();
        calculateQPLowerBound();
        calculateQPUpperBound();
    }

    void OSQP_MPCSolver::calculateQPHessian()
    {
        const int Nn = config_.window_size;
        const int Nx = 13;
        std::vector<Triplet<double>> triplets;
        for (int i = 0; i < Nn; i++)
        {
            const int idx = Nx * i;
            for (int j = 0; j < Nx; j++)
                for (int k = 0; k < Nx; k++)
                {
                    const double value = Q_[i](j, k);
                    if (value != 0)
                        triplets.push_back(Triplet<double>(idx + k, idx + k, value));
                }
        }
        int Nu_i = 0;
        for (int i = 0; i < Nn; i++)
        {
            const int Nu = 3 * n_legs_[i];
            const int idx = Nx * (Nn + 1) + Nu_i;
            for (int j = 0; j < Nu; j++)
                for (int k = 0; k < Nu; k++)
                {
                    const double value = R_[i](j, k);
                    if (value != 0)
                        triplets.push_back(Triplet<double>(idx + k, idx + k, value));
                }
            Nu_i += Nu;
        }
        H_.setFromTriplets(triplets.begin(), triplets.end());
    }

    void OSQP_MPCSolver::calculateQPGradient()
    {
        const int Nn = config_.window_size;
        const int Nx = 13;
        for (int i = 0; i < Nn + 1; i++)
        {
            const int xi = i * Nx;
            const VectorXd Gx = Q_[i].cast<double>() * -xref_[i].cast<double>();
            q_.block(xi, 0, Nx, 1) = Gx;
        }
        int Nu_i = 0;
        for (int i = 0; i < Nn; i++)
        {
            const int Nu = 3 * n_legs_[i];
            const int ui = Nx * (Nn + 1) + Nu_i;
            const VectorXd Gu = VectorXd::Zero(Nu);
            q_.block(ui, 0, Nu, 1) = Gu;
            Nu_i += Nu;
        }
    }

    void OSQP_MPCSolver::calculateQPLinearConstraint()
    {
        const int n = num_vars_;
        const int Nn = config_.window_size;
        const int Nx = 13;
        std::vector<Triplet<double>> triplets;
        for (int i = 0; i < Nx * (Nn + 1); i++)
            triplets.push_back(Triplet<double>(i, i, -1));

        int Nu_i = 0;
        for (int i = 0; i < Nn; i++)
            for (int j = 0; j < Nx; j++)
                for (int k = 0; k < Nx; k++)
                {
                    const double value = A_[i + 1](j, k);
                    if (value != 0)
                        triplets.push_back(Triplet<double>(
                            Nx * (i + 1) + j,
                            Nx * i + k,
                            value));
                }

        for (int i = 0; i < Nn; i++)
        {
            const int Nu = 3 * n_legs_[i];
            for (int j = 0; j < Nx; j++)
                for (int k = 0; k < Nu; k++)
                {
                    const double value = B_[i](j, k);
                    if (value != 0)
                        triplets.push_back(Triplet<double>(
                            Nx * (i + 1) + j,
                            Nu_i + k + Nx * (Nn + 1),
                            value));
                }
            Nu_i += Nu;
        }

        for (int i = 0; i < n; i++)
            triplets.push_back(Triplet<double>(i + (Nn + 1) * Nx, i, 1));

        Ac_.setFromTriplets(triplets.begin(), triplets.end());
    }

    void OSQP_MPCSolver::calculateQPLowerBound()
    {
        const int Nn = config_.window_size;
        const int Nx = 13;
        const int Neq = num_vars_;
        const int Nineq = num_constraints_ - num_vars_;

        VectorXd lower_inequality = VectorXd::Zero(Nineq);
        for (int i = 0; i < Nn + 1; i++)
            lower_inequality.block(Nx * i, 0, Nx, 1) = x_min_[i].cast<double>();

        int Nu_i = 0;
        for (int i = 0; i < Nn; i++)
        {
            const int Nu = 3 * n_legs_[i];
            lower_inequality.block(Nu_i + Nx * (Nn + 1), 0, Nu, 1) = u_min_[i].cast<double>();
            Nu_i += Nu;
        }

        VectorXd lower_equality = VectorXd::Zero(Neq);
        lower_equality.block(0, 0, Nx, 1) = -x0_.cast<double>();

        lc_.block(0, 0, Neq, 1) = lower_equality;
        lc_.block(Neq, 0, Nineq, 1) = lower_inequality;
    }

    void OSQP_MPCSolver::calculateQPUpperBound()
    {
        const int Nn = config_.window_size;
        const int Nx = 13;
        const int Neq = num_vars_;
        const int Nineq = num_constraints_ - num_vars_;

        VectorXd upper_inequality = VectorXd::Zero(Nineq);
        for (int i = 0; i < Nn + 1; i++)
            upper_inequality.block(Nx * i, 0, Nx, 1) = x_max_[i].cast<double>();

        int Nu_i = 0;
        for (int i = 0; i < Nn; i++)
        {
            const int Nu = 3 * n_legs_[i];
            upper_inequality.block(Nu * i + Nx * (Nn + 1), 0, Nu, 1) = u_max_[i].cast<double>();
            Nu_i += Nu;
        }

        VectorXd upper_equality = VectorXd::Zero(Neq);
        upper_equality.block(0, 0, Nx, 1) = -x0_.cast<double>();

        uc_.block(0, 0, Neq, 1) = upper_equality;
        uc_.block(Neq, 0, Nineq, 1) = upper_inequality;
    }

    void OSQP_MPCSolver::setupOSQP()
    {
        osqp_.n = num_vars_;
        osqp_.m = num_constraints_;

        osqp_.settings = new OSQPSettings;
        osqp_set_default_settings(osqp_.settings);

        osqp_.q = q_.data();
        osqp_.l = lc_.data();
        osqp_.u = uc_.data();

        convertEigenSparseToCSC(H_, osqp_.P, osqp_.Pnnz, osqp_.P->x, osqp_.P->i, osqp_.P->p);
        convertEigenSparseToCSC(Ac_, osqp_.A, osqp_.Annz, osqp_.A->x, osqp_.A->i, osqp_.A->p);

        osqp_setup(&osqp_.solver, osqp_.P, osqp_.q, osqp_.A, osqp_.l, osqp_.u, osqp_.m, osqp_.n, osqp_.settings);
    }

    void OSQP_MPCSolver::convertEigenSparseToCSC(const SparseMatrix<double> &matrix,
                                                 OSQPCscMatrix *&M, OSQPInt &Mnnz, OSQPFloat *&Mx, OSQPInt *&Mi, OSQPInt *&Mp)
    {
        M = new OSQPCscMatrix;
        Mnnz = matrix.nonZeros();
        Mx = new OSQPFloat[Mnnz];
        Mi = new OSQPInt[Mnnz];
        Mp = new OSQPInt[matrix.cols() + 1];

        int k = 0;
        Mp[0] = 0;
        for (int j = 0; j < matrix.outerSize(); ++j)
        {
            for (SparseMatrix<double>::InnerIterator it(matrix, j); it; ++it)
            {
                Mx[k] = it.value();
                Mi[k] = it.row();
                ++k;
            }
            Mp[j + 1] = k;
        }
        csc_set_data(M, matrix.rows(), matrix.cols(), Mnnz, Mx, Mi, Mp);
    }

}