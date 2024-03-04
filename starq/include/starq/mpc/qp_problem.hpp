#ifndef STARQ_MPC__QP_PROBLEM_HPP_
#define STARQ_MPC__QP_PROBLEM_HPP_

#include "starq/mpc/mpc_problem.hpp"
#include "eigen3/Eigen/Sparse"

namespace starq::mpc
{

    class QPProblem
    {
    public:
        using Ptr = std::shared_ptr<QPProblem>;

        QPProblem(const MPCProblem::Ptr mpc_problem);

        ~QPProblem();

        MPCProblem::Ptr getMPCProblem() const;

        void update();

        size_t getNx() const;

        size_t getNu() const;

        size_t getN() const;

        size_t getM() const;

        SparseMatrix<double> &getH();

        VectorXd &getG();

        SparseMatrix<double> &getAc();

        VectorXd &getLc();

        VectorXd &getUc();

    protected:
        const MPCProblem::Ptr mpc_problem_;

        size_t n_;
        size_t m_;

        size_t nx_;
        size_t nu_;

        SparseMatrix<double> H_;
        VectorXd g_;
        SparseMatrix<double> Ac_;
        VectorXd lc_;
        VectorXd uc_;

        void computeH();
        void computeG();
        void computeAc();
        void computeLc();
        void computeUc();
    };

}

#endif