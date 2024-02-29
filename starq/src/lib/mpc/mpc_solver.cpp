#include "starq/mpc/mpc_solver.hpp"

#include <iostream>

namespace starq::mpc
{

    MPCSolver::MPCSolver(const MPCConfiguration &config)
        : config_(config)
    {
        const int nn = config_.window_size;
        n_legs_.resize(nn);
        xref_.resize(nn);
        Q_.resize(nn);
        R_.resize(nn);
        A_.resize(nn);
        B_.resize(nn);
        x_min_.resize(nn);
        x_max_.resize(nn);
        u_min_.resize(nn);
        u_max_.resize(nn);

        initialize();
    }

    void MPCSolver::initialize()
    {
        VectorXf x0(13);
        x0.block<3, 1>(0, 0) = config_.com_trajectory[0].position;
        x0.block<3, 1>(3, 0) = config_.com_trajectory[0].orientation;
        x0.block<3, 1>(6, 0) = config_.com_trajectory[0].linear_velocity;
        x0.block<3, 1>(9, 0) = config_.com_trajectory[0].angular_velocity;
        x0(12) = 1.0;

        for (size_t k = 0; k < config_.window_size; k++)
        {

            VectorXf xref(13);
            xref.block<3, 1>(0, 0) = config_.com_trajectory[k].position;
            xref.block<3, 1>(3, 0) = config_.com_trajectory[k].orientation;
            xref.block<3, 1>(6, 0) = config_.com_trajectory[k].linear_velocity;
            xref.block<3, 1>(9, 0) = config_.com_trajectory[k].angular_velocity;
            xref(12) = 1.0;

            const float phi = config_.com_trajectory[k].orientation.z();
            const Matrix3f Rz = AngleAxisf(phi, Vector3f::UnitZ()).toRotationMatrix();
            const Vector3f g = config_.gravity;

            MatrixXf A(13, 13);
            A.block<3, 3>(0, 6) = Rz;
            A.block<3, 3>(3, 9) = Matrix3f::Identity();
            A.block<1, 3>(12, 9) = g.transpose();

            int n_legs;
            for (size_t i = 0; i < config_.stance_trajectory[k].size(); i++)
            {
                if (config_.stance_trajectory[k][i])
                {
                    n_legs++;
                }
            }

            const float inv_m = 1.0 / config_.mass;
            const Matrix3f I = Rz * config_.inertia * Rz.transpose();
            const Matrix3f inv_I = I.inverse();

            MatrixXf B(13, 3 * n_legs);
            for (int j = 0; j < n_legs; j++)
            {
                const Vector3f foot_position = config_.foothold_trajectory[k][j];
                const Vector3f r = foot_position - config_.com_trajectory[k].position;
                const Matrix3f skew_r = getSkewSymmetricMatrix(r);
                const Matrix3f inv_I_skew_r = inv_I * skew_r;
                B.block<3, 3>(6, 3 * j) = inv_I_skew_r;
                B.block<3, 3>(9, 3 * j) = inv_m * Matrix3f::Identity();
            }

            MatrixXf Q(13, 13);
            Q.block<3, 3>(0, 0) = config_.position_weights.asDiagonal();
            Q.block<3, 3>(3, 3) = config_.orientation_weights.asDiagonal();
            Q.block<3, 3>(6, 6) = config_.linear_velocity_weights.asDiagonal();
            Q.block<3, 3>(9, 9) = config_.angular_velocity_weights.asDiagonal();
            Q(12, 12) = 0.0;

            MatrixXf R(3 * n_legs, 3 * n_legs);
            for (int j = 0; j < n_legs; j++)
            {
                R.block<3, 3>(3 * j, 3 * j) = config_.force_weights.asDiagonal();
            }

            VectorXf x_min(13), x_max(13);
            for (int i = 0; i < 12; i++)
            {
                x_min(i) = -std::numeric_limits<float>::infinity();
                x_max(i) = std::numeric_limits<float>::infinity();
            }
            x_min(12) = 1.0;
            x_max(12) = 1.0;

            VectorXf u_min(3 * n_legs), u_max(3 * n_legs);
            for (int j = 0; j < n_legs; j++)
            {
                u_min.block<3, 1>(3 * j, 0) = config_.force_min;
                u_max.block<3, 1>(3 * j, 0) = config_.force_max;
            }

            n_legs_[k] = n_legs;
            xref_[k] = xref;
            Q_[k] = Q;
            R_[k] = R;
            A_[k] = A;
            B_[k] = B;
            x_min_[k] = x_min;
            x_max_[k] = x_max;
            u_min_[k] = u_min;
            u_max_[k] = u_max;
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

    void MPCSolver::print()
    {
        std::cout << "MPC Solver" << std::endl;
        for (size_t k = 0; k < config_.window_size; k++)
        {
            std::cout << "k = " << k << std::endl;
            std::cout << "n_legs = " << n_legs_[k] << std::endl;
            std::cout << "xref = " << xref_[k].transpose() << std::endl;
            std::cout << "Q = " << Q_[k] << std::endl;
            std::cout << "R = " << R_[k] << std::endl;
            std::cout << "A = " << A_[k] << std::endl;
            std::cout << "B = " << B_[k] << std::endl;
            std::cout << "x_min = " << x_min_[k].transpose() << std::endl;
            std::cout << "x_max = " << x_max_[k].transpose() << std::endl;
            std::cout << "u_min = " << u_min_[k].transpose() << std::endl;
            std::cout << "u_max = " << u_max_[k].transpose() << std::endl;
        }
    }

}