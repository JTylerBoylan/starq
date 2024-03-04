#include "starq/osqp/osqp.hpp"

#include <iostream>

namespace starq::osqp
{

    OSQP::OSQP(const mpc::QPProblem::Ptr qp_problem)
        : qp_problem_(qp_problem)
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

    void OSQP::update()
    {
        qp_problem_->update();

        n_ = qp_problem_->getN();
        m_ = qp_problem_->getM();

        q_ = qp_problem_->getG().data();
        l_ = qp_problem_->getLc().data();
        u_ = qp_problem_->getUc().data();

        convertEigenSparseToCSC(qp_problem_->getH(), P_, Pnnz_, Px_, Pi_, Pp_);
        convertEigenSparseToCSC(qp_problem_->getAc(), A_, Annz_, Ax_, Ai_, Ap_);

        osqp_cleanup(solver_);
        osqp_setup(&solver_, P_, q_, A_, l_, u_, m_, n_, settings_);
    }

    void OSQP::solve()
    {
        osqp_solve(solver_);
    }

    starq::mpc::MPCSolution OSQP::getSolution() const
    {
        starq::mpc::MPCSolution solution;
        solution.run_time = std::chrono::microseconds(static_cast<int>(solver_->info->run_time * 1e6));
        solution.setup_time = std::chrono::microseconds(static_cast<int>(solver_->info->setup_time * 1e6));
        solution.solve_time = std::chrono::microseconds(static_cast<int>(solver_->info->solve_time * 1e6));

        const auto config = qp_problem_->getMPCProblem()->getConfig();
        const size_t window_size = config->getWindowSize();
        const auto x = solver_->solution->x;
        for (size_t i = 0; i < window_size; i++)
        {
            const int ox = 13 * i;
            Vector3f orientation(x[ox], x[ox + 1], x[ox + 2]);
            Vector3f position(x[ox + 3], x[ox + 4], x[ox + 5]);
            Vector3f angular_velocity(x[ox + 6], x[ox + 7], x[ox + 8]);
            Vector3f linear_velocity(x[ox + 9], x[ox + 10], x[ox + 11]);
            starq::mpc::ReferenceState state = {position, orientation, linear_velocity, angular_velocity};
            solution.x_star.push_back(state);
        }

        int offset = qp_problem_->getMPCProblem()->getNx();
        for (size_t i = 0; i < window_size - 1; i++)
        {
            starq::mpc::FootForceState forces;
            const auto n_legs = config->getNumberOfLegs(i);
            for (size_t j = 0; j < n_legs; j++)
            {
                if (config->getStanceState(i)[j])
                {
                    Vector3f force(x[offset], x[offset + 1], x[offset + 2]);
                    forces.push_back(std::make_pair(true, force));
                    offset += 3;
                }
                else
                {
                    forces.push_back(std::make_pair(false, Vector3f::Zero()));
                }
            }
            solution.u_star.push_back(forces);
        }

        return solution;
    }

    void OSQP::convertEigenSparseToCSC(const SparseMatrix<double> &matrix,
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