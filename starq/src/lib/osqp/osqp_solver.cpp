#include "starq/osqp/osqp_solver.hpp"

#include <iostream>

namespace starq::osqp
{

    OSQP_MPCSolver::OSQP_MPCSolver(const MPCConfiguration &config)
        : MPCSolver(config)
    {
        osqp_.settings = new OSQPSettings;
        osqp_set_default_settings(osqp_.settings);
    }

    OSQP_MPCSolver::~OSQP_MPCSolver()
    {
        osqp_cleanup(osqp_.solver);
        delete osqp_.P;
        delete osqp_.A;
        delete osqp_.settings;
    }

    bool OSQP_MPCSolver::setup()
    {
        setupQP();
        setupOSQP();
        return true;
    }

    bool OSQP_MPCSolver::solve()
    {
        return osqp_solve(osqp_.solver) == 0;
    }

    void OSQP_MPCSolver::setupQP()
    {
        calculateAqp();
        calculateBqp();
        calculateL();
        calculateK();
        calculateY();

        calculateQPHessian();
        calculateQPGradient();
        calculateQPLinearConstraint();
        calculateQPLowerBound();
        calculateQPUpperBound();
    }

    void OSQP_MPCSolver::calculateAqp()
    {
        Aqp_ = MatrixXf::Zero(nx_, 13);
        Aqp_.block<13, 13>(0, 0) = MatrixXf::Identity(13, 13);
        for (size_t i = 1; i < config_.window_size; i++)
        {
            Aqp_.block<13, 13>(i * 13, 0) = A_[i - 1] * Aqp_.block<13, 13>((i - 1) * 13, 0);
            std::cout << "A" << i - 1 << ": " << A_[i - 1] << std::endl;
        }
        std::cout << "Aqp: " << Aqp_ << std::endl;
        std::cout << "Aqp * x0" << Aqp_ * xref_[0] << std::endl;
    }

    void OSQP_MPCSolver::calculateBqp()
    {
        Bqp_ = MatrixXf::Zero(nx_, nu_);
        int col_idx = 0;
        for (size_t i = 0; i < config_.window_size - 1; i++)
        {
            const int nf = 3 * n_legs_[i];
            Bqp_.block((i + 1) * 13, col_idx, 13, nf) = B_[i];

            for (size_t j = i; j < config_.window_size - 2; j++)
            {
                Bqp_.block((j + 2) * 13, col_idx, 13, nf) = A_[j + 1] * Bqp_.block((j + 1) * 13, col_idx, 13, nf);
            }

            col_idx += nf;

            std::cout << "B" << i << ": " << B_[i] << std::endl;
        }
        std::cout << "Bqp: " << Bqp_ << std::endl;
    }

    void OSQP_MPCSolver::calculateL()
    {
        L_ = MatrixXf::Zero(nx_, nx_);
        for (size_t i = 0; i < config_.window_size; i++)
        {
            L_.block(i * 13, i * 13, 13, 13) = Q_[i];
        }
        std::cout << "L: " << L_ << std::endl;
    }

    void OSQP_MPCSolver::calculateK()
    {
        K_ = MatrixXf::Zero(nu_, nu_);
        int u_idx = 0;
        for (size_t i = 0; i < config_.window_size - 1; i++)
        {
            const int nf = 3 * n_legs_[i];
            K_.block(u_idx, u_idx, nf, nf) = R_[i];
            u_idx += nf;
        }
        std::cout << "K: " << K_ << std::endl;
    }

    void OSQP_MPCSolver::calculateY()
    {
        y_ = VectorXf::Zero(nx_);
        for (size_t i = 0; i < config_.window_size; i++)
        {
            y_.block<13, 1>(i * 13, 0) = xref_[i];
        }
        std::cout << "y: " << y_ << std::endl;
    }

    void OSQP_MPCSolver::calculateQPHessian()
    {
        MatrixXf H = 2 * (Bqp_.transpose() * L_ * Bqp_ + K_);
        H = H.triangularView<Eigen::Upper>();
        H_ = H.sparseView().cast<double>();
        std::cout << "Hessian: " << H_ << std::endl;
    }

    void OSQP_MPCSolver::calculateQPGradient()
    {
        VectorXf q = 2 * Bqp_.transpose() * L_ * (Aqp_ * xref_[0] - y_);
        q_ = q.cast<double>();
        std::cout << "Gradient: " << q_ << std::endl;
    }

    void OSQP_MPCSolver::calculateQPLinearConstraint()
    {
        MatrixXf Ac = MatrixXf::Zero(2 * nu_, nu_);
        int idx = 0;
        for (size_t i = 0; i < config_.window_size - 1; i++)
        {
            const int nf = 3 * n_legs_[i];
            Ac.block(2 * idx, idx, 2 * nf, nf) = C_[i];
            idx += nf;
        }
        Ac_ = Ac.cast<double>().sparseView();
        std::cout << "Linear constraint: " << Ac_ << std::endl;
    }

    void OSQP_MPCSolver::calculateQPLowerBound()
    {
        VectorXf lc = VectorXf::Zero(2 * nu_);
        int idx = 0;
        for (size_t i = 0; i < config_.window_size - 1; i++)
        {
            const int nf = 3 * n_legs_[i];
            lc.block(2 * idx, 0, 2 * nf, 1) = cl_[i];
            idx += nf;
        }
        lc_ = lc.cast<double>();
        std::cout << "Lower bound: " << lc_ << std::endl;
    }

    void OSQP_MPCSolver::calculateQPUpperBound()
    {
        VectorXf uc = VectorXf::Zero(2 * nu_);
        int idx = 0;
        for (size_t i = 0; i < config_.window_size - 1; i++)
        {
            const int nf = 3 * n_legs_[i];
            uc.block(2 * idx, 0, 2 * nf, 1) = cu_[i];
            idx += nf;
        }
        uc_ = uc.cast<double>();
        std::cout << "Upper bound: " << uc_ << std::endl;
    }

    void OSQP_MPCSolver::setupOSQP()
    {
        osqp_.n = H_.rows();
        osqp_.m = Ac_.rows();

        osqp_.q = q_.data();
        osqp_.l = lc_.data();
        osqp_.u = uc_.data();

        convertEigenSparseToCSC(H_, osqp_.P, osqp_.Pnnz, osqp_.Px, osqp_.Pi, osqp_.Pp);
        convertEigenSparseToCSC(Ac_, osqp_.A, osqp_.Annz, osqp_.Ax, osqp_.Ai, osqp_.Ap);

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