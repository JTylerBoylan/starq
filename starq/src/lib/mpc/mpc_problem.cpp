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

    VectorXf &MPCProblem::getXref(const size_t &k)
    {
        return xref_[k];
    }

    MatrixXf &MPCProblem::getQ(const size_t &k)
    {
        return Q_[k];
    }

    MatrixXf &MPCProblem::getR(const size_t &k)
    {
        return R_[k];
    }

    MatrixXf &MPCProblem::getA(const size_t &k)
    {
        return A_[k];
    }

    MatrixXf &MPCProblem::getB(const size_t &k)
    {

        return B_[k];
    }

    MatrixXf &MPCProblem::getC(const size_t &k)
    {
        return C_[k];
    }

    VectorXf &MPCProblem::getLower(const size_t &k)
    {
        return lower_[k];
    }

    VectorXf &MPCProblem::getUpper(const size_t &k)
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
            VectorXf xref = VectorXf::Zero(13);
            xref.block<3, 1>(0, 0) = state.orientation;
            xref.block<3, 1>(3, 0) = state.position;
            xref.block<3, 1>(6, 0) = state.angular_velocity;
            xref.block<3, 1>(9, 0) = state.linear_velocity;
            xref(12) = 1.0;
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
            MatrixXf Q = MatrixXf::Zero(13, 13);
            Q.block<3, 3>(0, 0) = weights.orientation.asDiagonal();
            Q.block<3, 3>(3, 3) = weights.position.asDiagonal();
            Q.block<3, 3>(6, 6) = weights.angular_velocity.asDiagonal();
            Q.block<3, 3>(9, 9) = weights.linear_velocity.asDiagonal();
            Q(12, 12) = 0.0;
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
            MatrixXf R = MatrixXf::Zero(3 * n_legs, 3 * n_legs);
            for (size_t j = 0; j < n_legs; j++)
            {
                R.block<3, 3>(3 * j, 3 * j) = weights.asDiagonal();
            }
            R_[k] = R;
        }
    }

    void MPCProblem::computeA()
    {
        const size_t Nn = config_->getWindowSize();
        A_.resize(Nn - 1);
        for (size_t k = 0; k < Nn - 1; k++)
        {
            const float phi = config_->getReferenceState(k).orientation.z();
            const Matrix3f Rz = AngleAxisf(phi, Vector3f::UnitZ()).toRotationMatrix();
            const Vector3f g = config_->getGravity();

            MatrixXf A = MatrixXf::Zero(13, 13);
            A.block<3, 3>(0, 6) = Rz;
            A.block<3, 3>(3, 9) = Matrix3f::Identity();
            A.block<3, 1>(9, 12) = g;
            A = A * config_->getTimeStep() + MatrixXf::Identity(13, 13);
            A_[k] = A;
        }
    }

    void MPCProblem::computeB()
    {
        const size_t Nn = config_->getWindowSize();
        B_.resize(Nn - 1);
        for (size_t k = 0; k < Nn - 1; k++)
        {
            const float inv_m = 1.0 / config_->getMass();
            const float phi = config_->getReferenceState(k).orientation.z();
            const Matrix3f Rz = AngleAxisf(phi, Vector3f::UnitZ()).toRotationMatrix();
            const Matrix3f I = Rz * config_->getInertia() * Rz.transpose();
            const Matrix3f inv_I = I.inverse();

            const size_t n_legs = config_->getNumberOfLegs(k);
            const StanceState &stance = config_->getStanceState(k);

            MatrixXf B = MatrixXf::Zero(13, 3 * n_legs);
            int l = 0;
            for (size_t j = 0; j < stance.size(); j++)
            {
                if (stance[j])
                {
                    const Vector3f foot_position = config_->getFootholdState(k)[j];
                    const Vector3f r = foot_position - config_->getReferenceState(k).position;
                    const Matrix3f skew_r = getSkewSymmetricMatrix(r);
                    const Matrix3f inv_I_skew_r = inv_I * skew_r;
                    B.block<3, 3>(6, 3 * l) = inv_I_skew_r;
                    B.block<3, 3>(9, 3 * l) = inv_m * Matrix3f::Identity();
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
            const float mu = config_->getFrictionCoefficient();
            const size_t n_legs = config_->getNumberOfLegs(k);
            MatrixXf C = MatrixXf::Zero(6 * n_legs, 3 * n_legs);
            for (size_t j = 0; j < n_legs; j++)
            {
                C.block<6, 3>(6 * j, 3 * j) << 0, 0, -1,
                    0, 0, 1,
                    -1, 0, -mu,
                    1, 0, -mu,
                    0, -1, -mu,
                    0, 1, -mu;
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
            const float inf = std::numeric_limits<float>::infinity();
            VectorXf cl_l = VectorXf::Zero(6);
            cl_l << -inf, -inf, -inf, -inf, -inf, -inf;
            VectorXf cl = VectorXf::Zero(6 * n_legs);
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
            VectorXf cu_l = VectorXf::Zero(6);
            cu_l << -config_->getForceZMin(), config_->getForceZMax(), 0, 0, 0, 0;
            VectorXf cu = VectorXf::Zero(6 * n_legs);
            for (size_t j = 0; j < n_legs; j++)
            {
                cu.block<6, 1>(6 * j, 0) = cu_l;
            }
            upper_[k] = cu;
        }
    }

    Matrix3f MPCProblem::getSkewSymmetricMatrix(const Vector3f &v) const
    {
        Matrix3f S;
        S << 0, -v.z(), v.y(),
            v.z(), 0, -v.x(),
            -v.y(), v.x(), 0;
        return S;
    }

}