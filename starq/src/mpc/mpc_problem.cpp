#include "starq/mpc/mpc_problem.hpp"

#include <iostream>

namespace starq::mpc
{

    MPCProblem::MPCProblem()
    {
    }

    MPCProblem::~MPCProblem()
    {
    }

    MPCConfiguration::Ptr MPCProblem::getConfig() const
    {
        return config_;
    }

    bool MPCProblem::update(MPCConfiguration::Ptr config)
    {
        config_ = config;

        if (!config_->update())
        {
            std::cerr << "Failed to update the MPC configuration" << std::endl;
            return false;
        }

        computeXref();
        computeQ();
        computeR();
        computeA();
        computeB();
        computeC();
        computeLower();
        computeUpper();

        return true;
    }

    VectorX &MPCProblem::getXref(const size_t &k)
    {
        return xref_[k];
    }

    MatrixX &MPCProblem::getQ(const size_t &k)
    {
        return Q_[k];
    }

    MatrixX &MPCProblem::getR(const size_t &k)
    {
        return R_[k];
    }

    MatrixX &MPCProblem::getA(const size_t &k)
    {
        return A_[k];
    }

    MatrixX &MPCProblem::getB(const size_t &k)
    {

        return B_[k];
    }

    MatrixX &MPCProblem::getC(const size_t &k)
    {
        return C_[k];
    }

    VectorX &MPCProblem::getLower(const size_t &k)
    {
        return lower_[k];
    }

    VectorX &MPCProblem::getUpper(const size_t &k)
    {
        return upper_[k];
    }

    void MPCProblem::computeXref()
    {
        const size_t Nn = config_->getWindowSize();
        xref_.resize(Nn);
        for (size_t k = 0; k < Nn; k++)
        {
            const ReferenceState &state = config_->getReferenceState(k);
            VectorX xref = VectorX::Zero(7);
            xref.block<3, 1>(0, 0) = state.angular_velocity;
            xref.block<3, 1>(3, 0) = state.linear_velocity;
            xref(6) = 1.0;
            xref_[k] = xref;
        }
    }

    void MPCProblem::computeQ()
    {
        const size_t Nn = config_->getWindowSize();
        Q_.resize(Nn);
        for (size_t k = 0; k < Nn; k++)
        {
            const ReferenceWeights &weights = config_->getReferenceWeights(k);
            MatrixX Q = MatrixX::Zero(7, 7);
            Q.block<3, 3>(0, 0) = weights.angular_velocity.asDiagonal();
            Q.block<3, 3>(3, 3) = weights.linear_velocity.asDiagonal();
            Q(6, 6) = 0.0;
            Q_[k] = Q;
        }
    }

    void MPCProblem::computeR()
    {
        const size_t Nn = config_->getWindowSize();
        R_.resize(Nn - 1);
        for (size_t k = 0; k < Nn - 1; k++)
        {
            const size_t n_legs = config_->getNumberOfLegs(k);
            const ForceWeights &weights = config_->getForceWeights(k);
            MatrixX R = MatrixX::Zero(3 * n_legs, 3 * n_legs);
            for (size_t j = 0; j < n_legs; j++)
            {
                R.block<3, 3>(3 * j, 3 * j) = weights.asDiagonal();
            }
            R_[k] = R;
        }
    }

    void MPCProblem::computeA()
    {
        MatrixX A = MatrixX::Zero(7, 7);
        A.block<3, 1>(3, 6) = config_->getGravity();
        A = A * config_->getTimeStep() + MatrixX::Identity(7, 7);
        const size_t Nn = config_->getWindowSize();
        A_.resize(Nn - 1, A);
    }

    void MPCProblem::computeB()
    {
        const size_t Nn = config_->getWindowSize();
        B_.resize(Nn - 1);
        for (size_t k = 0; k < Nn - 1; k++)
        {
            const Float inv_m = 1.0 / config_->getMass();
            const Matrix3 I = config_->getInertia();
            const Matrix3 inv_I = I.inverse();

            const size_t n_legs = config_->getNumberOfLegs(k);
            const StanceState &stance = config_->getStanceState(k);

            MatrixX B = MatrixX::Zero(7, 3 * n_legs);
            int l = 0;
            for (size_t j = 0; j < stance.size(); j++)
            {
                if (stance[j])
                {
                    const Vector3 r = config_->getFootholdState(k)[j];
                    const Matrix3 skew_r = getSkewSymmetricMatrix(r);
                    const Matrix3 inv_I_skew_r = inv_I * skew_r;
                    B.block<3, 3>(0, 3 * l) = inv_I_skew_r;
                    B.block<3, 3>(3, 3 * l) = inv_m * Matrix3::Identity();
                    l++;
                }
            }
            B = B * config_->getTimeStep();
            B_[k] = B;
        }
    }

    void MPCProblem::computeC()
    {
        const size_t Nn = config_->getWindowSize();
        C_.resize(Nn - 1);
        for (size_t k = 0; k < Nn - 1; k++)
        {
            const Float mu = config_->getFrictionCoefficient();
            const Float mu_y = config_->is2D() ? 0.0 : mu;
            const size_t n_legs = config_->getNumberOfLegs(k);
            MatrixX C = MatrixX::Zero(6 * n_legs, 3 * n_legs);
            for (size_t j = 0; j < n_legs; j++)
            {
                C.block<6, 3>(6 * j, 3 * j) << 0, 0, -1,
                    0, 0, 1,
                    -1, 0, -mu,
                    1, 0, -mu,
                    0, -1, -mu_y,
                    0, 1, -mu_y;
            }
            C_[k] = C;
        }
    }

    void MPCProblem::computeLower()
    {
        const size_t Nn = config_->getWindowSize();
        lower_.resize(Nn - 1);
        for (size_t k = 0; k < Nn - 1; k++)
        {
            const size_t n_legs = config_->getNumberOfLegs(k);
            const Float inf = std::numeric_limits<Float>::infinity();
            VectorX cl_l = VectorX::Zero(6);
            cl_l << -inf, -inf, -inf, -inf, -inf, -inf;
            VectorX cl = VectorX::Zero(6 * n_legs);
            for (size_t j = 0; j < n_legs; j++)
            {
                cl.block<6, 1>(6 * j, 0) = cl_l;
            }
            lower_[k] = cl;
        }
    }

    void MPCProblem::computeUpper()
    {
        const size_t Nn = config_->getWindowSize();
        upper_.resize(Nn - 1);
        for (size_t k = 0; k < Nn - 1; k++)
        {
            const size_t n_legs = config_->getNumberOfLegs(k);
            VectorX cu_l = VectorX::Zero(6);
            cu_l << -config_->getForceZMin(), config_->getForceZMax(), 0, 0, 0, 0;
            VectorX cu = VectorX::Zero(6 * n_legs);
            for (size_t j = 0; j < n_legs; j++)
            {
                cu.block<6, 1>(6 * j, 0) = cu_l;
            }
            upper_[k] = cu;
        }
    }

    Matrix3 MPCProblem::getSkewSymmetricMatrix(const Vector3 &v) const
    {
        Matrix3 S;
        S << 0, -v.z(), v.y(),
            v.z(), 0, -v.x(),
            -v.y(), v.x(), 0;
        return S;
    }

}