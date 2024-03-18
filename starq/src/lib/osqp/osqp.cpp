#include "starq/osqp/osqp.hpp"

#include <iostream>

namespace starq::osqp
{

    OSQP::OSQP()
        : mpc::MPCSolver(),
          qp_problem_(std::make_shared<mpc::QPProblem>())
    {
        settings_ = new OSQPSettings;
        osqp_set_default_settings(settings_);
    }

    OSQP::~OSQP()
    {
        osqp_cleanup(solver_);
        delete P_;
        delete A_;
        delete settings_;
    }

    bool OSQP::update(mpc::MPCConfiguration::Ptr config)
    {
        if (!qp_problem_->update(config))
        {
            std::cerr << "Failed to update the QP problem" << std::endl;
            return false;
        }

        n_ = qp_problem_->getN();
        m_ = qp_problem_->getM();

        const Eigen::VectorXd &q = qp_problem_->getG().cast<double>();
        const Eigen::VectorXd &l = qp_problem_->getLc().cast<double>();
        const Eigen::VectorXd &u = qp_problem_->getUc().cast<double>();

        if (q_ != nullptr)
            delete[] q_;
        if (l_ != nullptr)
            delete[] l_;
        if (u_ != nullptr)
            delete[] u_;

        q_ = new OSQPFloat[n_];
        l_ = new OSQPFloat[m_];
        u_ = new OSQPFloat[m_];

        std::memcpy(q_, q.data(), n_ * sizeof(OSQPFloat));
        std::memcpy(l_, l.data(), m_ * sizeof(OSQPFloat));
        std::memcpy(u_, u.data(), m_ * sizeof(OSQPFloat));

        convertEigenSparseToCSC(qp_problem_->getH(), P_, Pnnz_, Px_, Pi_, Pp_);
        convertEigenSparseToCSC(qp_problem_->getAc(), A_, Annz_, Ax_, Ai_, Ap_);

        OSQPInt flag = osqp_cleanup(solver_);
        if (flag != 0)
        {
            std::cerr << "Error cleaning up OSQP solver (Error: " << flag << ")" << std::endl;
            return false;
        }

        flag = osqp_setup(&solver_, P_, q_, A_, l_, u_, m_, n_, settings_);
        if (flag != 0)
        {
            std::cerr << "Error setting up OSQP solver (Error: " << flag << ")" << std::endl;
            return false;
        }

        return true;
    }

    bool OSQP::solve()
    {
        OSQPInt flag = osqp_solve(solver_);
        if (flag != 0)
        {
            std::cerr << "Error solving OSQP problem (Error: " << flag << ")" << std::endl;
            return false;
        }
        
        saveMPCSolution();
        return true;
    }

    starq::mpc::MPCSolution::Ptr OSQP::getSolution() const
    {
        return solution_;
    }

    starq::mpc::FootForceState OSQP::getFirstForceState() const
    {
        if (solution_ == nullptr)
            return starq::mpc::FootForceState();
        return solution_->u_star[0];
    }

    void OSQP::convertEigenSparseToCSC(const Eigen::SparseMatrix<Float> &matrix,
                                       OSQPCscMatrix *&M, OSQPInt &Mnnz, OSQPFloat *&Mx, OSQPInt *&Mi, OSQPInt *&Mp)
    {
        if (M != nullptr)
            delete M;
        if (Mx != nullptr)
            delete[] Mx;
        if (Mi != nullptr)
            delete[] Mi;
        if (Mp != nullptr)
            delete[] Mp;

        M = new OSQPCscMatrix;
        Mnnz = matrix.nonZeros();
        Mx = new OSQPFloat[Mnnz];
        Mi = new OSQPInt[Mnnz];
        Mp = new OSQPInt[matrix.cols() + 1];

        int k = 0;
        Mp[0] = 0;
        for (int j = 0; j < matrix.outerSize(); ++j)
        {
            for (Eigen::SparseMatrix<Float>::InnerIterator it(matrix, j); it; ++it)
            {
                Mx[k] = static_cast<OSQPFloat>(it.value());
                Mi[k] = static_cast<OSQPFloat>(it.row());
                ++k;
            }
            Mp[j + 1] = k;
        }
        csc_set_data(M, matrix.rows(), matrix.cols(), Mnnz, Mx, Mi, Mp);
    }

    void OSQP::saveMPCSolution()
    {
        if (solution_ == nullptr)
            solution_ = std::make_shared<starq::mpc::MPCSolution>();

        solution_->run_time = std::chrono::microseconds(static_cast<int>(solver_->info->run_time * 1e6));
        solution_->setup_time = std::chrono::microseconds(static_cast<int>(solver_->info->setup_time * 1e6));
        solution_->solve_time = std::chrono::microseconds(static_cast<int>(solver_->info->solve_time * 1e6));

        const auto config = qp_problem_->getMPCProblem()->getConfig();
        const size_t window_size = config->getWindowSize();

        solution_->x_star.resize(window_size);
        const auto x = solver_->solution->x;
        for (size_t i = 0; i < window_size; i++)
        {
            const int ox = 13 * i;
            Vector3 orientation(x[ox], x[ox + 1], x[ox + 2]);
            Vector3 position(x[ox + 3], x[ox + 4], x[ox + 5]);
            Vector3 angular_velocity(x[ox + 6], x[ox + 7], x[ox + 8]);
            Vector3 linear_velocity(x[ox + 9], x[ox + 10], x[ox + 11]);
            starq::mpc::ReferenceState state = {position, orientation, linear_velocity, angular_velocity};
            solution_->x_star[i] = state;
        }

        solution_->u_star.resize(window_size - 1);
        int offset = qp_problem_->getNx();
        for (size_t i = 0; i < window_size - 1; i++)
        {
            starq::mpc::FootForceState forces;
            const auto n_legs = config->getStanceState(i).size();

            forces.reserve(n_legs);
            for (size_t j = 0; j < n_legs; j++)
            {
                if (config->getStanceState(i)[j])
                {
                    Vector3 force(x[offset], x[offset + 1], x[offset + 2]);
                    forces.push_back(std::make_pair(true, force));
                    offset += 3;
                }
                else
                {
                    forces.push_back(std::make_pair(false, Vector3::Zero()));
                }
            }
            solution_->u_star[i] = forces;
        }
    }

}