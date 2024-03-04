#include "starq/mpc/qp_problem.hpp"

#include <iostream>

namespace starq::mpc
{

    QPProblem::QPProblem(const MPCProblem::Ptr mpc_problem)
        : mpc_problem_(mpc_problem)
    {
    }

    QPProblem::~QPProblem()
    {
    }

    MPCProblem::Ptr QPProblem::getMPCProblem() const
    {
        return mpc_problem_;
    }

    size_t QPProblem::getN() const
    {
        return n_;
    }

    size_t QPProblem::getM() const
    {
        return m_;
    }

    void QPProblem::update()
    {
        mpc_problem_->update();

        n_ = mpc_problem_->getNx() + mpc_problem_->getNu();
        m_ = mpc_problem_->getNx() + 2 * mpc_problem_->getNu();
    }

    SparseMatrix<double>& QPProblem::getH()
    {
        H_.resize(n_, n_);

        const int Nn = mpc_problem_->getConfig()->getWindowSize();
        const int Nx = 13;
        std::vector<Triplet<double>> triplets;

        for (int i = 0; i < Nn; i++)
        {
            const MatrixXf Qi = mpc_problem_->getQ(i);
            const int idx = Nx * i;
            for (int j = 0; j < Nx; j++)
                for (int k = 0; k < Nx; k++)
                {
                    const double value = Qi(j, k);
                    if (value != 0)
                        triplets.push_back(Triplet<double>(idx + j, idx + k, value));
                }
        }

        int idx = Nx * Nn;
        for (int i = 0; i < Nn - 1; i++)
        {
            const MatrixXf Ri = mpc_problem_->getR(i);
            const int Nu = 3 * mpc_problem_->getConfig()->getNumberOfLegs(i);
            for (int j = 0; j < Nu; j++)
                for (int k = 0; k < Nu; k++)
                {
                    const double value = Ri(j, k);
                    if (value != 0)
                        triplets.push_back(Triplet<double>(idx + j, idx + k, value));
                }
            idx += Nu;
        }

        H_.setFromTriplets(triplets.begin(), triplets.end());
        return H_;
    }

    VectorXd& QPProblem::getG()
    {
        g_.resize(n_);

        const int Nn = mpc_problem_->getConfig()->getWindowSize();
        const int Nx = 13;

        for (int i = 0; i < Nn; i++)
        {
            const int xi = i * Nx;
            const VectorXd Gx = mpc_problem_->getQ(i).cast<double>() * -mpc_problem_->getXref(i).cast<double>();
            g_.block(xi, 0, Nx, 1) = Gx;
        }

        int idx = Nx * Nn;
        for (int i = 0; i < Nn - 1; i++)
        {
            const int Nu = 3 * mpc_problem_->getConfig()->getNumberOfLegs(i);
            const VectorXd Gu = VectorXd::Zero(Nu);
            g_.block(idx, 0, Nu, 1) = Gu;
            idx += Nu;
        }

        return g_;
    }

    SparseMatrix<double>& QPProblem::getAc()
    {
        Ac_.resize(m_, n_);

        const int Nn = mpc_problem_->getConfig()->getWindowSize();
        const int Nx = 13;

        std::vector<Triplet<double>> triplets;
        for (int i = 0; i < Nx * Nn; i++)
            triplets.push_back(Triplet<double>(i, i, -1));

        for (int i = 0; i < Nn - 1; i++)
        {
            const MatrixXf Ai = mpc_problem_->getA(i);
            for (int j = 0; j < Nx; j++)
                for (int k = 0; k < Nx; k++)
                {
                    const double value = Ai(j, k);
                    if (value != 0)
                        triplets.push_back(Triplet<double>(
                            Nx * (i + 1) + j,
                            Nx * i + k,
                            value));
                }
        }

        int idx = Nx * Nn;
        for (int i = 0; i < Nn - 1; i++)
        {
            const MatrixXf Bi = mpc_problem_->getB(i);
            const int Nu = 3 * mpc_problem_->getConfig()->getNumberOfLegs(i);
            for (int j = 0; j < Nx; j++)
                for (int k = 0; k < Nu; k++)
                {
                    const double value = Bi(j, k);
                    if (value != 0)
                        triplets.push_back(Triplet<double>(
                            Nx * (i + 1) + j,
                            idx + k,
                            value));
                }
            idx += Nu;
        }

        idx = 0;
        for (int i = 0; i < Nn - 1; i++)
        {
            const MatrixXf Ci = mpc_problem_->getC(i);
            const int Nu = 3 * mpc_problem_->getConfig()->getNumberOfLegs(i);
            for (int j = 0; j < 2 * Nu; j++)
                for (int k = 0; k < Nu; k++)
                {
                    const double value = Ci(j, k);
                    if (value != 0)
                        triplets.push_back(Triplet<double>(
                            Nx * Nn + 2 * idx + j,
                            Nx * Nn + idx + k,
                            value));
                }
            idx += Nu;
        }

        Ac_.setFromTriplets(triplets.begin(), triplets.end());
        return Ac_;
    }

    VectorXd& QPProblem::getLc()
    {
        lc_.resize(m_);

        const int Nn = mpc_problem_->getConfig()->getWindowSize();
        const int Nx = 13;
        const int Neq = mpc_problem_->getNx();
        const int Nineq = 2 * mpc_problem_->getNu();

        lc_.block(0, 0, Neq, 1) = VectorXd::Zero(Neq);
        lc_.block(0, 0, Nx, 1) = -mpc_problem_->getXref(0).cast<double>();

        lc_.block(Neq, 0, Nineq, 1) = VectorXd::Zero(Nineq);
        int idx = Neq;
        for (int i = 0; i < Nn - 1; i++)
        {
            const int Nuc = 6 * mpc_problem_->getConfig()->getNumberOfLegs(i);
            lc_.block(idx, 0, Nuc, 1) = mpc_problem_->getLower(i).cast<double>();
            idx += Nuc;
        }

        return lc_;
    }

    VectorXd& QPProblem::getUc()
    {
        uc_.resize(m_);

        const int Nn = mpc_problem_->getConfig()->getWindowSize();
        const int Nx = 13;
        const int Neq = mpc_problem_->getNx();
        const int Nineq = 2 * mpc_problem_->getNu();

        uc_.block(0, 0, Neq, 1) = VectorXd::Zero(Neq);
        uc_.block(0, 0, Nx, 1) = -mpc_problem_->getXref(0).cast<double>();

        uc_.block(Neq, 0, Nineq, 1) = VectorXd::Zero(Nineq);
        int idx = Neq;
        for (int i = 0; i < Nn - 1; i++)
        {
            const int Nuc = 6 * mpc_problem_->getConfig()->getNumberOfLegs(i);
            uc_.block(idx, 0, Nuc, 1) = mpc_problem_->getUpper(i).cast<double>();
            idx += Nuc;
        }

        return uc_;
    }

}