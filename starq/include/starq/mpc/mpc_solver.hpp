#ifndef STARQ_MPC__MPC_SOLVER_HPP_
#define STARQ_MPC__MPC_SOLVER_HPP_

#include "starq/mpc/mpc_types.hpp"

namespace starq::mpc 
{

    class MPCSolver 
    {
    public:
        using Ptr = std::shared_ptr<MPCSolver>;

        MPCSolver(const MPCConfiguration &config);

        virtual bool solve() = 0;

    protected:

        void initialize();

        Matrix3f getSkewSymmetricMatrix(const Vector3f &v);

        const MPCConfiguration config_;

        VectorXf x0_;
        std::vector<int> n_legs_;
        std::vector<VectorXf> xref_;
        std::vector<MatrixXf> Q_;
        std::vector<MatrixXf> R_;
        std::vector<MatrixXf> A_;
        std::vector<MatrixXf> B_;
        std::vector<VectorXf> x_min_;
        std::vector<VectorXf> x_max_;
        std::vector<VectorXf> u_min_;
        std::vector<VectorXf> u_max_;
    };

}

#endif