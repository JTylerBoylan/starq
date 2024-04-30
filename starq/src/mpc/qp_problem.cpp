#include "starq/mpc/qp_problem.hpp"

#include <iostream>

namespace starq::mpc
{

    QPProblem::QPProblem()
        : mpc_problem_(std::make_shared<MPCProblem>()),
          nx_(0),
          nu_(0),
          n_(0),
          m_(0)
    {
    }

    QPProblem::~QPProblem()
    {
    }

    MPCProblem::Ptr QPProblem::getMPCProblem() const
    {
        return mpc_problem_;
    }

    size_t QPProblem::getNx() const
    {
        return nx_;
    }

    size_t QPProblem::getNu() const
    {
        return nu_;
    }

    size_t QPProblem::getN() const
    {
        return n_;
    }

    size_t QPProblem::getM() const
    {
        return m_;
    }

    bool QPProblem::update(MPCConfiguration::Ptr config)
    {
        if (!mpc_problem_->update(config))
        {
            std::cerr << "Failed to update the MPC problem" << std::endl;
            return false;
        }

        const size_t Nn = mpc_problem_->getConfig()->getWindowSize();
        nx_ = 7 * Nn;
        nu_ = 0;
        for (size_t k = 0; k < Nn - 1; k++)
        {
            nu_ += 3 * mpc_problem_->getConfig()->getNumberOfLegs(k);
        }

        n_ = nx_ + nu_;
        m_ = nx_ + 2 * nu_;

        computeH();
        computeG();
        computeAc();
        computeLc();
        computeUc();

        return true;
    }

    Eigen::SparseMatrix<Float> &QPProblem::getH()
    {
        return H_;
    }

    VectorX &QPProblem::getG()
    {
        return g_;
    }

    Eigen::SparseMatrix<Float> &QPProblem::getAc()
    {
        return Ac_;
    }

    VectorX &QPProblem::getLc()
    {
        return lc_;
    }

    VectorX &QPProblem::getUc()
    {
        return uc_;
    }

    void QPProblem::computeH()
    {
        H_.resize(n_, n_);

        const int Nn = mpc_problem_->getConfig()->getWindowSize();
        const int Nx = 7;
        std::vector<Eigen::Triplet<Float>> triplets;

        for (int i = 0; i < Nn; i++)
        {
            const MatrixX Qi = mpc_problem_->getQ(i);
            const int idx = Nx * i;
            for (int j = 0; j < Nx; j++)
                for (int k = 0; k < Nx; k++)
                {
                    const Float value = Qi(j, k);
                    if (value != 0)
                        triplets.push_back(Eigen::Triplet<Float>(idx + j, idx + k, value));
                }
        }

        int idx = Nx * Nn;
        for (int i = 0; i < Nn - 1; i++)
        {
            const MatrixX Ri = mpc_problem_->getR(i);
            const int Nu = 3 * mpc_problem_->getConfig()->getNumberOfLegs(i);
            for (int j = 0; j < Nu; j++)
                for (int k = 0; k < Nu; k++)
                {
                    const Float value = Ri(j, k);
                    if (value != 0)
                        triplets.push_back(Eigen::Triplet<Float>(idx + j, idx + k, value));
                }
            idx += Nu;
        }

        H_.setFromTriplets(triplets.begin(), triplets.end());
    }

    void QPProblem::computeG()
    {
        g_.resize(n_);

        const int Nn = mpc_problem_->getConfig()->getWindowSize();
        const int Nx = 7;

        for (int i = 0; i < Nn; i++)
        {
            const int xi = i * Nx;
            const VectorX Gx = mpc_problem_->getQ(i) * -mpc_problem_->getXref(i);
            g_.block(xi, 0, Nx, 1) = Gx;
        }

        int idx = Nx * Nn;
        for (int i = 0; i < Nn - 1; i++)
        {
            const int Nu = 3 * mpc_problem_->getConfig()->getNumberOfLegs(i);
            const VectorX Gu = VectorX::Zero(Nu);
            g_.block(idx, 0, Nu, 1) = Gu;
            idx += Nu;
        }
    }

    void QPProblem::computeAc()
    {
        Ac_.resize(m_, n_);

        const int Nn = mpc_problem_->getConfig()->getWindowSize();
        const int Nx = 7;

        std::vector<Eigen::Triplet<Float>> triplets;
        for (int i = 0; i < Nx * Nn; i++)
            triplets.push_back(Eigen::Triplet<Float>(i, i, -1));

        for (int i = 0; i < Nn - 1; i++)
        {
            const MatrixX Ai = mpc_problem_->getA(i);
            for (int j = 0; j < Nx; j++)
                for (int k = 0; k < Nx; k++)
                {
                    const Float value = Ai(j, k);
                    if (value != 0)
                        triplets.push_back(Eigen::Triplet<Float>(
                            Nx * (i + 1) + j,
                            Nx * i + k,
                            value));
                }
        }

        int idx = Nx * Nn;
        for (int i = 0; i < Nn - 1; i++)
        {
            const MatrixX Bi = mpc_problem_->getB(i);
            const int Nu = 3 * mpc_problem_->getConfig()->getNumberOfLegs(i);
            for (int j = 0; j < Nx; j++)
                for (int k = 0; k < Nu; k++)
                {
                    const Float value = Bi(j, k);
                    if (value != 0)
                        triplets.push_back(Eigen::Triplet<Float>(
                            Nx * (i + 1) + j,
                            idx + k,
                            value));
                }
            idx += Nu;
        }

        idx = 0;
        for (int i = 0; i < Nn - 1; i++)
        {
            const MatrixX Ci = mpc_problem_->getC(i);
            const int Nu = 3 * mpc_problem_->getConfig()->getNumberOfLegs(i);
            for (int j = 0; j < 2 * Nu; j++)
                for (int k = 0; k < Nu; k++)
                {
                    const Float value = Ci(j, k);
                    if (value != 0)
                        triplets.push_back(Eigen::Triplet<Float>(
                            Nx * Nn + 2 * idx + j,
                            Nx * Nn + idx + k,
                            value));
                }
            idx += Nu;
        }

        Ac_.setFromTriplets(triplets.begin(), triplets.end());
    }

    void QPProblem::computeLc()
    {
        lc_.resize(m_);

        const int Nn = mpc_problem_->getConfig()->getWindowSize();
        const int Nx = 7;
        const int Neq = nx_;
        const int Nineq = 2 * nu_;

        lc_.block(0, 0, Neq, 1) = VectorX::Zero(Neq);
        lc_.block(0, 0, Nx, 1) = -mpc_problem_->getXref(0);

        lc_.block(Neq, 0, Nineq, 1) = VectorX::Zero(Nineq);
        int idx = Neq;
        for (int i = 0; i < Nn - 1; i++)
        {
            const int Nuc = 6 * mpc_problem_->getConfig()->getNumberOfLegs(i);
            lc_.block(idx, 0, Nuc, 1) = mpc_problem_->getLower(i);
            idx += Nuc;
        }
    }

    void QPProblem::computeUc()
    {
        uc_.resize(m_);

        const int Nn = mpc_problem_->getConfig()->getWindowSize();
        const int Nx = 7;
        const int Neq = nx_;
        const int Nineq = 2 * nu_;

        uc_.block(0, 0, Neq, 1) = VectorX::Zero(Neq);
        uc_.block(0, 0, Nx, 1) = -mpc_problem_->getXref(0);

        uc_.block(Neq, 0, Nineq, 1) = VectorX::Zero(Nineq);
        int idx = Neq;
        for (int i = 0; i < Nn - 1; i++)
        {
            const int Nuc = 6 * mpc_problem_->getConfig()->getNumberOfLegs(i);
            uc_.block(idx, 0, Nuc, 1) = mpc_problem_->getUpper(i);
            idx += Nuc;
        }
    }

}