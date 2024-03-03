#include "starq/mpc/mpc_solver.hpp"

#include <iostream>

namespace starq::mpc
{

    MPCSolver::MPCSolver(const MPCConfiguration &config)
        : config_(config)
    {
        const int nn = config_.window_size;
        n_legs_.resize(nn - 1);
        xref_.resize(nn);
        Q_.resize(nn);
        R_.resize(nn - 1);
        A_.resize(nn - 1);
        B_.resize(nn - 1);
        C_.resize(nn - 1);
        cl_.resize(nn - 1);
        cu_.resize(nn - 1);

        initialize();
    }

    void MPCSolver::initialize()
    {
        const size_t nn = config_.window_size;

        nx_ = 13 * nn;
        nu_ = 0;

        for (size_t k = 0; k < nn; k++)
        {
            VectorXf xref = VectorXf::Zero(13);
            xref.block<3, 1>(0, 0) = config_.com_trajectory[k].orientation;
            xref.block<3, 1>(3, 0) = config_.com_trajectory[k].position;
            xref.block<3, 1>(6, 0) = config_.com_trajectory[k].angular_velocity;
            xref.block<3, 1>(9, 0) = config_.com_trajectory[k].linear_velocity;
            xref(12) = 1.0;

            MatrixXf Q = MatrixXf::Zero(13, 13);
            Q.block<3, 3>(0, 0) = config_.orientation_weights.asDiagonal();
            Q.block<3, 3>(3, 3) = config_.position_weights.asDiagonal();
            Q.block<3, 3>(6, 6) = config_.angular_velocity_weights.asDiagonal();
            Q.block<3, 3>(9, 9) = config_.linear_velocity_weights.asDiagonal();
            Q(12, 12) = 0.0;

            xref_[k] = xref;
            Q_[k] = Q;
        }

        for (size_t k = 0; k < nn - 1; k++)
        {
            const float phi = config_.com_trajectory[k].orientation.z();
            const Matrix3f Rz = AngleAxisf(phi, Vector3f::UnitZ()).toRotationMatrix();
            const Vector3f g = config_.gravity;

            MatrixXf A = MatrixXf::Zero(13, 13);
            A.block<3, 3>(0, 6) = Rz;
            A.block<3, 3>(3, 9) = Matrix3f::Identity();
            A.block<3, 1>(9, 12) = g;
            A = A * config_.time_step.count() * 1E-3 + MatrixXf::Identity(13, 13);

            int n_legs = 0;
            for (size_t j = 0; j < config_.stance_trajectory[k].size(); j++)
            {
                if (config_.stance_trajectory[k][j])
                {
                    n_legs++;
                    nu_ += 3;
                }
            }

            const float inv_m = 1.0 / config_.mass;
            const Matrix3f I = Rz * config_.inertia * Rz.transpose();
            const Matrix3f inv_I = I.inverse();

            const float mu = config_.friction_coeff;
            MatrixXf C_l(6, 3);
            C_l << 0, 0, -1,
                0, 0, 1,
                -1, 0, -mu,
                1, 0, -mu,
                0, -1, -mu,
                0, 1, -mu;

            const float inf = std::numeric_limits<float>::infinity();
            VectorXf cl_l(6), cu_l(6);
            cl_l << -inf, -inf, -inf, -inf, -inf, -inf;
            cu_l << -config_.fz_min, config_.fz_max, 0, 0, 0, 0;

            MatrixXf B = MatrixXf::Zero(13, 3 * n_legs);
            MatrixXf C = MatrixXf::Zero(6 * n_legs, 3 * n_legs);
            VectorXf cl = VectorXf::Zero(6 * n_legs);
            VectorXf cu = VectorXf::Zero(6 * n_legs);
            int l = 0;
            for (size_t j = 0; j < config_.stance_trajectory[k].size(); j++)
            {
                if (config_.stance_trajectory[k][j])
                {
                    const Vector3f foot_position = config_.foothold_trajectory[k][j];
                    const Vector3f r = foot_position - config_.com_trajectory[k].position;
                    const Matrix3f skew_r = getSkewSymmetricMatrix(r);
                    const Matrix3f inv_I_skew_r = inv_I * skew_r;
                    B.block<3, 3>(6, 3 * l) = inv_I_skew_r;
                    B.block<3, 3>(9, 3 * l) = inv_m * Matrix3f::Identity();
                    C.block<6, 3>(6 * l, 3 * l) = C_l;
                    cl.block<6, 1>(6 * l, 0) = cl_l;
                    cu.block<6, 1>(6 * l, 0) = cu_l;
                    l++;
                }
            }
            B = B * config_.time_step.count() * 1E-3;

            MatrixXf R = MatrixXf::Zero(3 * n_legs, 3 * n_legs);
            for (int j = 0; j < n_legs; j++)
            {
                R.block<3, 3>(3 * j, 3 * j) = config_.force_weights.asDiagonal();
            }

            n_legs_[k] = n_legs;
            R_[k] = R;
            A_[k] = A;
            B_[k] = B;
            C_[k] = C;
            cu_[k] = cu;
            cl_[k] = cl;
        }
    }

    Matrix3f MPCSolver::getSkewSymmetricMatrix(const Vector3f &v)
    {
        Matrix3f S;
        S << 0, -v.z(), v.y(),
            v.z(), 0, -v.x(),
            -v.y(), v.x(), 0;
        return S;
    }

}