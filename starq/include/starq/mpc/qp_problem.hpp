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

        virtual size_t getN() const;

        virtual size_t getM() const;

        virtual SparseMatrix<double>& getH();

        virtual VectorXd& getG();

        virtual SparseMatrix<double>& getAc();

        virtual VectorXd& getLc();

        virtual VectorXd& getUc();

    protected:
        const MPCProblem::Ptr mpc_problem_;

        size_t n_;
        size_t m_;

        SparseMatrix<double> H_;
        VectorXd g_;
        SparseMatrix<double> Ac_;
        VectorXd lc_;
        VectorXd uc_;
    };

}

#endif