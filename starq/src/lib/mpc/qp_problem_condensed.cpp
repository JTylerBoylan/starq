#include "starq/mpc/qp_problem_condensed.hpp"

#include <iostream>

namespace starq::mpc
{

    QPProblemCondensed::QPProblemCondensed(const MPCProblem::Ptr mpc_problem)
        : QPProblem(mpc_problem)
    {
    }

    QPProblemCondensed::~QPProblemCondensed()
    {
    }

    size_t QPProblemCondensed::getN() const
    {
        return mpc_problem_->getNu();
    }

    size_t QPProblemCondensed::getM() const
    {
        return 2 * mpc_problem_->getNu();
    }

    SparseMatrix<double> &QPProblemCondensed::getH()
    {
        const auto Bqp = getBqp();
        const auto L = getL();
        const auto K = getK();
        MatrixXf H = 2 * (Bqp.transpose() * L * Bqp + K);
        H = H.triangularView<Eigen::Upper>();
        H_ = H.sparseView().cast<double>();
        return H_;
    }

    VectorXd &QPProblemCondensed::getG()
    {
        const auto Aqp = getAqp();
        const auto Bqp = getBqp();
        const auto L = getL();
        const auto y = getY();
        const auto x0 = mpc_problem_->getXref(0);
        VectorXf q = 2 * Bqp.transpose() * L * (Aqp * x0 - y);
        g_ = q.cast<double>();
        return g_;
    }

    SparseMatrix<double> &QPProblemCondensed::getAc()
    {
        MatrixXf Ac = MatrixXf::Zero(2 * mpc_problem_->getNu(), mpc_problem_->getNu());
        int idx = 0;
        for (size_t i = 0; i < mpc_problem_->getConfig()->getWindowSize() - 1; i++)
        {
            const int nf = 3 * mpc_problem_->getConfig()->getNumberOfLegs(i);
            const auto C = mpc_problem_->getC(i);
            Ac.block(2 * idx, idx, 2 * nf, nf) = C;
            idx += nf;
        }
        Ac_ = Ac.cast<double>().sparseView();
        return Ac_;
    }

    VectorXd &QPProblemCondensed::getLc()
    {
        VectorXf lc = VectorXf::Zero(2 * mpc_problem_->getNu());
        int idx = 0;
        for (size_t i = 0; i < mpc_problem_->getConfig()->getWindowSize() - 1; i++)
        {
            const int nf = 3 * mpc_problem_->getConfig()->getNumberOfLegs(i);
            auto cl = mpc_problem_->getLower(i);
            lc.block(2 * idx, 0, 2 * nf, 1) = cl;
            idx += nf;
        }
        lc_ = lc.cast<double>();
        return lc_;
    }

    VectorXd &QPProblemCondensed::getUc()
    {
        VectorXf uc = VectorXf::Zero(2 * mpc_problem_->getNu());
        int idx = 0;
        for (size_t i = 0; i < mpc_problem_->getConfig()->getWindowSize() - 1; i++)
        {
            const int nf = 3 * mpc_problem_->getConfig()->getNumberOfLegs(i);
            auto cu = mpc_problem_->getUpper(i);
            uc.block(2 * idx, 0, 2 * nf, 1) = cu;
            idx += nf;
        }
        uc_ = uc.cast<double>();
        return uc_;
    }

    MatrixXf QPProblemCondensed::getAqp()
    {
        MatrixXf Aqp = MatrixXf::Zero(mpc_problem_->getNx(), 13);
        Aqp.block<13, 13>(0, 0) = MatrixXf::Identity(13, 13);
        for (size_t i = 1; i < mpc_problem_->getConfig()->getWindowSize(); i++)
        {
            const auto A = mpc_problem_->getA(i - 1);
            Aqp.block<13, 13>(i * 13, 0) = A * Aqp.block<13, 13>((i - 1) * 13, 0);
        }
        return Aqp;
    }

    MatrixXf QPProblemCondensed::getBqp()
    {
        MatrixXf Bqp = MatrixXf::Zero(mpc_problem_->getNx(), mpc_problem_->getNu());

        const size_t window_size = mpc_problem_->getConfig()->getWindowSize();
        int col_idx = 0;
        for (size_t i = 0; i < window_size - 1; i++)
        {
            const int nf = 3 * mpc_problem_->getConfig()->getNumberOfLegs(i);
            const auto B = mpc_problem_->getB(i);

            Bqp.block((i + 1) * 13, col_idx, 13, nf) = B;

            for (size_t j = i; j < window_size - 2; j++)
            {
                const auto A = mpc_problem_->getA(j + 1); // FIX THIS (getA() recomputes)
                Bqp.block((j + 2) * 13, col_idx, 13, nf) = A * Bqp.block((j + 1) * 13, col_idx, 13, nf);
            }

            col_idx += nf;
        }
        return Bqp;
    }

    MatrixXf QPProblemCondensed::getL()
    {
        MatrixXf L = MatrixXf::Zero(mpc_problem_->getNx(), mpc_problem_->getNx());
        for (size_t i = 0; i < mpc_problem_->getConfig()->getWindowSize(); i++)
        {
            auto Q = mpc_problem_->getQ(i);
            L.block(i * 13, i * 13, 13, 13) = Q;
        }
        return L;
    }

    MatrixXf QPProblemCondensed::getK()
    {
        MatrixXf K = MatrixXf::Zero(mpc_problem_->getNu(), mpc_problem_->getNu());
        int u_idx = 0;
        for (size_t i = 0; i < mpc_problem_->getConfig()->getWindowSize() - 1; i++)
        {
            const int nf = 3 * mpc_problem_->getConfig()->getNumberOfLegs(i);
            auto R = mpc_problem_->getR(i);
            K.block(u_idx, u_idx, nf, nf) = R;
            u_idx += nf;
        }
        return K;
    }

    VectorXf QPProblemCondensed::getY()
    {
        VectorXf y = VectorXf::Zero(mpc_problem_->getNx());
        for (size_t i = 0; i < mpc_problem_->getConfig()->getWindowSize(); i++)
        {
            y.block<13, 1>(i * 13, 0) = mpc_problem_->getXref(i);
        }
        return y;
    }

}