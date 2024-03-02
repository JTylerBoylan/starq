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

        x_min_.resize(nn);
        x_max_.resize(nn);

        u_min_.resize(nn - 1);
        u_max_.resize(nn - 1);

        initialize();
    }

    void MPCSolver::initialize()
    {
        const size_t nn = config_.window_size;
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

            VectorXf x_min = VectorXf::Zero(13);
            VectorXf x_max = VectorXf::Zero(13);
            for (int i = 0; i < 12; i++)
            {
                x_min(i) = -std::numeric_limits<float>::infinity();
                x_max(i) = std::numeric_limits<float>::infinity();
            }
            x_min(12) = 1.0;
            x_max(12) = 1.0;

            xref_[k] = xref;
            Q_[k] = Q;
            x_min_[k] = x_min;
            x_max_[k] = x_max;
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

            int n_legs = 0;
            for (size_t j = 0; j < config_.stance_trajectory[k].size(); j++)
            {
                if (config_.stance_trajectory[k][j])
                {
                    n_legs++;
                }
            }

            const float inv_m = 1.0 / config_.mass;
            const Matrix3f I = Rz * config_.inertia * Rz.transpose();
            const Matrix3f inv_I = I.inverse();

            MatrixXf B = MatrixXf::Zero(13, 3 * n_legs);
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
                    l++;
                }
            }

            MatrixXf R = MatrixXf::Zero(3 * n_legs, 3 * n_legs);
            for (int j = 0; j < n_legs; j++)
            {
                R.block<3, 3>(3 * j, 3 * j) = config_.force_weights.asDiagonal();
            }

            VectorXf u_min = VectorXf::Zero(3 * n_legs);
            VectorXf u_max = VectorXf::Zero(3 * n_legs);
            for (int j = 0; j < n_legs; j++)
            {
                u_min.block<3, 1>(3 * j, 0) = config_.force_min;
                u_max.block<3, 1>(3 * j, 0) = config_.force_max;
            }

            n_legs_[k] = n_legs;
            R_[k] = R;
            A_[k] = A;
            B_[k] = B;
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

}