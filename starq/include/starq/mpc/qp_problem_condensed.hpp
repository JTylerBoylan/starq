#ifndef STARQ_MPC__QP_PROBLEM_CONDENSED_HPP_
#define STARQ_MPC__QP_PROBLEM_CONDENSED_HPP_

#include "starq/mpc/qp_problem.hpp"

namespace starq::mpc
{

    class QPProblemCondensed : public QPProblem
    {
    public:
        using Ptr = std::shared_ptr<QPProblemCondensed>;

        QPProblemCondensed(const MPCProblem::Ptr mpc_problem);

        ~QPProblemCondensed();

        size_t getN() const override;

        size_t getM() const override;

        SparseMatrix<double> &getH() override;

        VectorXd &getG() override;

        SparseMatrix<double> &getAc() override;

        VectorXd &getLc() override;

        VectorXd &getUc() override;

    private:

        MatrixXf getAqp();
        MatrixXf getBqp();
        MatrixXf getL();
        MatrixXf getK();
        VectorXf getY();
    };

}

#endif